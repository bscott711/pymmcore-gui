"""Frame sources for the spot focus lock: the ToupTek camera and a fake.

Used inside the camera worker subprocess only. Both sources follow the
:class:`FrameSource` protocol, so the worker never knows which one it has.

- :class:`ToupcamSource` -- the AmScope MD310C-BS (ToupTek OEM) through the
  vendor's ctypes wrapper. The wrapper is **not bundled**: it is loaded from
  ``sdk_dir`` or ``$TOUPCAM_SDK_DIR`` when :meth:`ToupcamSource.open` runs.
- :class:`FakeSpotSource` -- renders a Gaussian spot that follows the commanded
  piezo position plus drift, so the whole loop can be tested without hardware.
- :func:`make_source` -- build either from a plain dict (picklable for spawn).
"""

from __future__ import annotations

import importlib.util
import logging
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any, Protocol

import numpy as np

logger = logging.getLogger(__name__)

SDK_DIR_ENV = "TOUPCAM_SDK_DIR"
_SDK_MODULES = ("toupcam", "amcam")  # amcam.py is the AmScope-branded alias


class FrameSource(Protocol):
    """What the worker needs from a camera."""

    def open(self) -> None:
        """Connect and start streaming."""

    def close(self) -> None:
        """Stop streaming and release the camera. Safe to call twice."""

    def latest(self, timeout_s: float) -> tuple[np.ndarray, float] | None:
        """Return the newest raw frame and its ``time.monotonic()`` stamp.

        Older frames are dropped. None if no new frame arrived in *timeout_s*.
        """

    def set_exposure_ms(self, ms: float) -> None:
        """Set a fixed exposure time (auto-exposure stays off)."""

    def set_gain(self, gain: int) -> None:
        """Set the analogue gain in percent (100 = 1x)."""

    def set_roi(self, x: int, y: int, w: int, h: int) -> None:
        """Restrict readout to a hardware ROI in full-sensor pixels."""

    def info(self) -> dict[str, Any]:
        """Return a small dict describing the source (model, size, ROI, ...)."""

    def set_actuator_um(self, p: float) -> None:
        """Tell the source the commanded piezo P. No-op for real cameras."""


# ---------------------------------------------------------------------------
# Real camera
# ---------------------------------------------------------------------------


def _load_sdk(sdk_dir: str | None) -> Any:
    """Import the vendor's ctypes wrapper from *sdk_dir* (or the env var)."""
    folder = sdk_dir or os.environ.get(SDK_DIR_ENV)
    if not folder:
        raise RuntimeError(
            f"ToupTek SDK not found: pass sdk_dir or set ${SDK_DIR_ENV} to the "
            "folder holding toupcam.py (or amcam.py) and its DLL"
        )
    for name in _SDK_MODULES:
        path = Path(folder) / f"{name}.py"
        if path.is_file():
            if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
                os.add_dll_directory(str(Path(folder).resolve()))
            spec = importlib.util.spec_from_file_location(f"_spot_{name}", path)
            assert spec is not None and spec.loader is not None
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module._spot_prefix = name  # type: ignore[attr-defined]
            return module
    raise RuntimeError(f"Neither toupcam.py nor amcam.py found in {folder}")


class ToupcamSource:
    """The ToupTek / AmScope camera in raw, fixed-exposure pull mode.

    SDK names below follow the standard ToupTek python wrapper and are UNVERIFIED
    against hardware: marked ``# SDK?`` where the exact spelling is a guess.
    """

    def __init__(self, serial: str | None = None, sdk_dir: str | None = None) -> None:
        self.serial = serial
        self.sdk_dir = sdk_dir
        self._sdk: Any = None
        self._cam: Any = None
        self._model = ""
        self._lock = threading.Lock()
        self._new = threading.Event()
        self._frame: tuple[np.ndarray, float] | None = None
        self._buf: bytes = b""
        self._size = (0, 0)
        self._roi = (0, 0, 0, 0)
        self._exposure_ms = 10.0
        self._gain = 100

    # -- helpers -----------------------------------------------------------

    def _const(self, suffix: str) -> Any:
        prefix = self._sdk._spot_prefix.upper()
        return getattr(self._sdk, f"{prefix}_{suffix}")

    def _alloc(self) -> None:
        w, h = self._cam.get_Size()
        self._size = (w, h)
        self._buf = bytes(w * h * 2)  # 16-bit samples; pull fills it in place

    def _on_event(self, event: int, _ctx: Any) -> None:
        """SDK callback (SDK thread): pull the newest frame into the 1-slot buffer."""
        if event != self._const("EVENT_IMAGE") or self._cam is None:
            return
        with self._lock:
            try:
                # bits=16 in raw mode: SDK? some sensors want the native bit depth
                self._cam.PullImageV2(self._buf, 16, None)
            except Exception:
                logger.exception("PullImageV2 failed")
                return
            w, h = self._size
            frame = np.frombuffer(self._buf, dtype=np.uint16).reshape(h, w).copy()
            self._frame = (frame, time.monotonic())
        self._new.set()

    # -- FrameSource -------------------------------------------------------

    def open(self) -> None:
        """Load the SDK, open the camera, start pull mode."""
        self._sdk = _load_sdk(self.sdk_dir)
        cls = getattr(self._sdk, self._sdk._spot_prefix.capitalize())
        devices = cls.EnumV2()
        if self.serial:
            devices = [d for d in devices if self.serial in d.id]
        if not devices:
            raise RuntimeError(
                f"No ToupTek camera found (serial={self.serial!r}); is it plugged "
                "in and not held by the vendor app?"
            )
        dev = devices[0]
        self._model = getattr(dev, "displayname", "") or ""
        cam = cls.Open(dev.id)
        if cam is None:
            raise RuntimeError(f"Open({dev.id}) failed")
        self._cam = cam
        cam.put_Option(self._const("OPTION_RAW"), 1)  # raw Bayer, no demosaic
        cam.put_AutoExpoEnable(0)
        cam.put_ExpoTime(int(self._exposure_ms * 1000))  # microseconds
        cam.put_ExpoAGain(int(self._gain))  # percent
        self._alloc()
        self._roi = (0, 0, *self._size)
        cam.StartPullModeWithCallback(self._on_event, self)

    def close(self) -> None:
        """Stop the camera; safe to call twice."""
        cam, self._cam = self._cam, None
        if cam is not None:
            try:
                cam.Close()
            except Exception:
                logger.exception("Closing the camera failed")
        self._new.clear()
        self._frame = None

    def latest(self, timeout_s: float) -> tuple[np.ndarray, float] | None:
        """Wait for the callback, then return and clear the 1-slot buffer."""
        if not self._new.wait(timeout_s):
            return None
        with self._lock:
            self._new.clear()
            out, self._frame = self._frame, None
        return out

    def set_exposure_ms(self, ms: float) -> None:
        """Set exposure in ms (SDK takes microseconds)."""
        self._exposure_ms = float(ms)
        if self._cam is not None:
            self._cam.put_ExpoTime(int(ms * 1000))

    def set_gain(self, gain: int) -> None:
        """Set analogue gain in percent."""
        self._gain = int(gain)
        if self._cam is not None:
            self._cam.put_ExpoAGain(int(gain))

    def set_roi(self, x: int, y: int, w: int, h: int) -> None:
        """Set the hardware ROI; the SDK wants even offsets and sizes."""
        if self._cam is None:
            raise RuntimeError("set_roi before open()")
        x, y, w, h = (int(v) // 2 * 2 for v in (x, y, w, h))
        with self._lock:
            self._cam.put_Roi(x, y, w, h)  # SDK? may need Stop/StartPullMode
            self._alloc()
            self._roi = (x, y, w, h)
            self._frame = None
        self._new.clear()

    def info(self) -> dict[str, Any]:
        """Describe the camera; ``roi`` is ``(x, y, w, h)`` of the frames."""
        return {
            "kind": "toupcam",
            "model": self._model,
            "serial": self.serial,
            "size": self._size,
            "roi": self._roi,
            "exposure_ms": self._exposure_ms,
            "gain": self._gain,
        }

    def set_actuator_um(self, p: float) -> None:
        """No-op: a real camera does not care."""


# ---------------------------------------------------------------------------
# Fake camera
# ---------------------------------------------------------------------------


class FakeSpotSource:
    """A simulated spot camera that closes the loop with the piezo.

    The spot is a Gaussian at ``x0 + ux*slope*(drift(t) + p - p0)`` (and the
    matching y), so a controller that moves P by ``-along_um`` pulls it back to
    the setpoint. *cal_like* is a plain dict with ``x0, y0, ux, uy,
    slope_px_per_um`` and ``p0_um`` (a ``SpotCalibration.to_dict()`` works).
    ``lateral_px_per_s`` drifts the spot perpendicular to ``u``, which focus
    cannot cancel. Drift accrues from :meth:`open`.
    """

    def __init__(
        self,
        cal_like: dict[str, Any],
        drift_um_per_s: float = 0.0,
        noise: float = 2.0,
        amplitude: float = 800.0,
        sigma_px: float = 6.0,
        shape: tuple[int, int] = (480, 640),
        lateral_px_per_s: float = 0.0,
        seed: int = 0,
        fps: float = 50.0,
    ) -> None:
        self.cal = dict(cal_like)
        self.drift_um_per_s = drift_um_per_s
        self.noise = noise
        self.amplitude = amplitude
        self.sigma_px = sigma_px
        self.shape = (int(shape[0]), int(shape[1]))
        self.lateral_px_per_s = lateral_px_per_s
        self.fps = fps
        self.offset = 100.0  # camera pedestal, counts
        self._rng = np.random.default_rng(seed)
        self._p = float(self.cal.get("p0_um", 0.0))
        self._t0: float | None = None
        self._next = 0.0
        self._roi = (0, 0, self.shape[1], self.shape[0])
        self._exposure_ms = 10.0
        self._gain = 100

    # -- model -------------------------------------------------------------

    def spot_position(self, t_s: float) -> tuple[float, float]:
        """Return the true (x, y) of the spot ``t_s`` seconds after open()."""
        c = self.cal
        along = (
            self.drift_um_per_s * t_s + self._p - float(c.get("p0_um", 0.0))
        ) * float(c["slope_px_per_um"])
        lateral = self.lateral_px_per_s * t_s
        ux, uy = float(c["ux"]), float(c["uy"])
        return (
            float(c["x0"]) + ux * along - uy * lateral,
            float(c["y0"]) + uy * along + ux * lateral,
        )

    def render(self, t_s: float) -> np.ndarray:
        """Render the (ROI-cropped) uint16 frame at time *t_s* after open()."""
        sx, sy = self.spot_position(t_s)
        rows, cols = self.shape
        img = np.full((rows, cols), self.offset, dtype=np.float64)
        s = self.sigma_px
        r0, r1 = max(int(sy - 6 * s), 0), min(int(sy + 6 * s) + 2, rows)
        c0, c1 = max(int(sx - 6 * s), 0), min(int(sx + 6 * s) + 2, cols)
        if r1 > r0 and c1 > c0:
            gy = np.exp(-((np.arange(r0, r1) - sy) ** 2) / (2 * s**2))
            gx = np.exp(-((np.arange(c0, c1) - sx) ** 2) / (2 * s**2))
            img[r0:r1, c0:c1] += self.amplitude * np.outer(gy, gx)
        if self.noise > 0:
            img += self._rng.normal(0.0, self.noise, img.shape)
        x, y, w, h = self._roi
        img = img[y : y + h, x : x + w]
        return np.clip(np.rint(img), 0, 65535).astype(np.uint16)

    # -- FrameSource -------------------------------------------------------

    def open(self) -> None:
        """Start the clock; drift accrues from here."""
        self._t0 = time.monotonic()
        self._next = self._t0

    def close(self) -> None:
        """Nothing to release."""

    def latest(self, timeout_s: float) -> tuple[np.ndarray, float] | None:
        """Sleep until the next frame is due, then render it."""
        if self._t0 is None:
            raise RuntimeError("latest() before open()")
        now = time.monotonic()
        if self._next - now > timeout_s:
            time.sleep(max(timeout_s, 0.0))
            return None
        if self._next > now:
            time.sleep(self._next - now)
        stamp = time.monotonic()
        # Skip frames we are late for: the newest one wins, as on a real camera.
        self._next = max(self._next + 1.0 / self.fps, stamp)
        return self.render(stamp - self._t0), stamp

    def set_exposure_ms(self, ms: float) -> None:
        """Stored only."""
        self._exposure_ms = float(ms)

    def set_gain(self, gain: int) -> None:
        """Stored only."""
        self._gain = int(gain)

    def set_roi(self, x: int, y: int, w: int, h: int) -> None:
        """Crop the rendered frames to the ROI (full-sensor pixels)."""
        self._roi = (int(x), int(y), int(w), int(h))

    def info(self) -> dict[str, Any]:
        """Describe the fake; ``roi`` is ``(x, y, w, h)`` of the frames."""
        return {
            "kind": "fake",
            "model": "FakeSpotSource",
            "size": (self.shape[1], self.shape[0]),
            "roi": self._roi,
            "exposure_ms": self._exposure_ms,
            "gain": self._gain,
        }

    def set_actuator_um(self, p: float) -> None:
        """Set the piezo P the spot follows."""
        self._p = float(p)


def make_source(spec: dict[str, Any]) -> FrameSource:
    """Build a source from a plain dict, so it can cross a spawn boundary.

    ``{"kind": "toupcam", "serial": ..., "sdk_dir": ...}`` or
    ``{"kind": "fake", "cal_like": {...}, <FakeSpotSource keyword args>}``.
    """
    kwargs = {k: v for k, v in spec.items() if k != "kind"}
    kind = spec.get("kind")
    if kind == "toupcam":
        return ToupcamSource(**kwargs)
    if kind == "fake":
        return FakeSpotSource(**kwargs)
    raise ValueError(f"Unknown frame source kind {kind!r}")


__all__ = [
    "SDK_DIR_ENV",
    "FakeSpotSource",
    "FrameSource",
    "ToupcamSource",
    "make_source",
]
