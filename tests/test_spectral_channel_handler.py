from __future__ import annotations

from pathlib import Path

import numpy as np
import useq
import zarr

from pymmcore_gui._settings import SpectralChannelConfig
from pymmcore_gui._spectral_channel_handler import (
    SpectralChannelHandler,
    channel_output_path,
)

LASER_GROUP = "Lasers"
ALL_LASERS = "AllLasers"
CAMERAS = ["Camera-1", "Camera-2"]


class _StubCore:
    """Minimal stand-in exposing only what SpectralChannelHandler needs."""

    def __init__(self, cameras: list[str]) -> None:
        self._cameras = cameras

    def getCameraDevice(self) -> str:
        return "MultiCam" if len(self._cameras) > 1 else self._cameras[0]


def _channels() -> list[SpectralChannelConfig]:
    """Two cameras, two splitter sub-regions each, on a 20x20 test frame."""
    return [
        SpectralChannelConfig(
            name="GFP_488",
            camera="Camera-1",
            laser_preset="488nm",
            rect=(0, 0, 20, 10),
            enabled=True,
        ),
        SpectralChannelConfig(
            name="CalceinViolet_405",
            camera="Camera-1",
            laser_preset="405nm",
            rect=(0, 10, 20, 10),
            enabled=True,
        ),
        SpectralChannelConfig(
            name="mScarlet_561",
            camera="Camera-2",
            laser_preset="561nm",
            rect=(0, 0, 20, 10),
            enabled=True,
        ),
        SpectralChannelConfig(
            name="CF647_638",
            camera="Camera-2",
            laser_preset="638nm",
            rect=(0, 10, 20, 10),
            enabled=True,
        ),
    ]


# Constant pixel value written for each (camera, region) pair, distinct enough
# to prove a channel's file only ever contains its own camera+region data.
_REGION_VALUE = {
    ("Camera-1", "top"): 11,
    ("Camera-1", "bottom"): 12,
    ("Camera-2", "top"): 21,
    ("Camera-2", "bottom"): 22,
}
_CHANNEL_EXPECTED_VALUE = {
    "GFP_488": _REGION_VALUE[("Camera-1", "top")],
    "CalceinViolet_405": _REGION_VALUE[("Camera-1", "bottom")],
    "mScarlet_561": _REGION_VALUE[("Camera-2", "top")],
    "CF647_638": _REGION_VALUE[("Camera-2", "bottom")],
}


def _make_frame(camera: str) -> np.ndarray:
    frame = np.zeros((20, 20), dtype=np.uint16)
    frame[:10, :] = _REGION_VALUE[(camera, "top")]
    frame[10:, :] = _REGION_VALUE[(camera, "bottom")]
    return frame


def _run(
    tmp_path: Path, channels: list[SpectralChannelConfig], presets: list[str]
) -> None:
    """Drive the handler through a sequence whose channel axis is *presets*."""
    seq = useq.MDASequence(
        channels=[useq.Channel(config=p, group=LASER_GROUP) for p in presets],
        z_plan=useq.ZRangeAround(range=1, step=1),  # 2 z slices
    )
    handler = SpectralChannelHandler(
        tmp_path / "acq.ome.zarr",
        channels,
        LASER_GROUP,
        ALL_LASERS,
        mmcore=_StubCore(CAMERAS),
    )
    handler.sequenceStarted(seq, {})
    for event in seq:
        for cam in CAMERAS:
            handler.frameReady(_make_frame(cam), event, {"camera_device": cam})
    handler.sequenceFinished(seq)


def test_channel_output_path() -> None:
    ch = SpectralChannelConfig(
        name="GFP 488",
        camera="Camera-1",
        laser_preset="488nm",
        writer_format="ome-zarr",
    )
    assert channel_output_path("d/exp.ome.tiff", ch) == "d/exp_GFP_488.ome.zarr"

    tiff_ch = ch.model_copy(update={"writer_format": "ome-tiff"})
    assert channel_output_path("d/exp", tiff_ch) == "d/exp_GFP_488.ome.tiff"


def test_single_laser_presets_route_each_channel_to_its_own_file(
    tmp_path: Path,
) -> None:
    """A sequence cycling the 4 individual laser presets writes 4 clean files."""
    channels = _channels()
    _run(tmp_path, channels, presets=["405nm", "488nm", "561nm", "638nm"])

    for ch in channels:
        out = Path(channel_output_path(tmp_path / "acq.ome.zarr", ch))
        assert out.exists(), f"missing output for {ch.name}: {out}"
        arr = zarr.open(str(out))["p0"]
        # Channel axis collapsed away: only (z, y, x) remain, fully populated
        # (both z slices), with no leftover empty/default-filled slots.
        assert arr.shape == (2, 10, 20)
        data = arr[:]
        assert int(data.min()) == int(data.max()) == _CHANNEL_EXPECTED_VALUE[ch.name]


def test_all_lasers_preset_writes_every_channel(tmp_path: Path) -> None:
    """A sequence using only the AllLasers preset still saves all 4 regions."""
    channels = _channels()
    _run(tmp_path, channels, presets=[ALL_LASERS])

    for ch in channels:
        out = Path(channel_output_path(tmp_path / "acq.ome.zarr", ch))
        assert out.exists(), f"missing output for {ch.name}: {out}"
        arr = zarr.open(str(out))["p0"]
        assert arr.shape == (2, 10, 20)
        data = arr[:]
        assert int(data.min()) == int(data.max()) == _CHANNEL_EXPECTED_VALUE[ch.name]


def test_channel_disabled_or_no_rect_is_never_passed_to_handler(
    tmp_path: Path,
) -> None:
    """Callers are expected to pre-filter to ``is_ready`` channels.

    Sanity-check ``is_ready`` itself, since the MDA-widget substitution point
    relies on it to build the ``channels`` list handed to the handler.
    """
    ch = SpectralChannelConfig(name="x", camera="Camera-1", laser_preset="488nm")
    assert not ch.is_ready  # no rect yet
    ch = ch.model_copy(update={"rect": (0, 0, 10, 10)})
    assert not ch.is_ready  # still disabled
    ch = ch.model_copy(update={"enabled": True})
    assert ch.is_ready
