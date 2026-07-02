from __future__ import annotations

from pathlib import Path

import numpy as np
import useq
import zarr

from pymmcore_gui._settings import SpectralChannelConfig
from pymmcore_gui._spectral_channel_handler import (
    SpectralChannelHandler,
    channel_output_path,
    channels_for_sequence,
    next_fft_friendly,
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
            name="GFP_488", camera="Camera-1", laser_preset="488nm", rect=(0, 0, 20, 10)
        ),
        SpectralChannelConfig(
            name="CalceinViolet_405",
            camera="Camera-1",
            laser_preset="405nm",
            rect=(0, 10, 20, 10),
        ),
        SpectralChannelConfig(
            name="mScarlet_561",
            camera="Camera-2",
            laser_preset="561nm",
            rect=(0, 0, 20, 10),
        ),
        SpectralChannelConfig(
            name="CF647_638",
            camera="Camera-2",
            laser_preset="638nm",
            rect=(0, 10, 20, 10),
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
    ch = SpectralChannelConfig(name="GFP 488", camera="Camera-1", laser_preset="488nm")
    # The format is supplied by the caller (the MDA save widget), not the config.
    assert (
        channel_output_path("d/exp.ome.tiff", ch, "ome-zarr")
        == "d/exp_GFP_488.ome.zarr"
    )
    assert channel_output_path("d/exp", ch, "ome-tiff") == "d/exp_GFP_488.ome.tiff"


def test_single_laser_presets_route_each_channel_to_its_own_file(
    tmp_path: Path,
) -> None:
    """A sequence cycling the 4 individual laser presets writes 4 clean files."""
    channels = _channels()
    _run(tmp_path, channels, presets=["405nm", "488nm", "561nm", "638nm"])

    for ch in channels:
        out = Path(channel_output_path(tmp_path / "acq.ome.zarr", ch, "ome-zarr"))
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
        out = Path(channel_output_path(tmp_path / "acq.ome.zarr", ch, "ome-zarr"))
        assert out.exists(), f"missing output for {ch.name}: {out}"
        arr = zarr.open(str(out))["p0"]
        assert arr.shape == (2, 10, 20)
        data = arr[:]
        assert int(data.min()) == int(data.max()) == _CHANNEL_EXPECTED_VALUE[ch.name]


def test_is_ready_requires_a_rect() -> None:
    """A channel is savable once it has a drawn rect (no per-channel flag)."""
    ch = SpectralChannelConfig(name="x", camera="Camera-1", laser_preset="488nm")
    assert not ch.is_ready  # no rect yet
    ch = ch.model_copy(update={"rect": (0, 0, 10, 10)})
    assert ch.is_ready


def _sequence(presets: list[str]) -> useq.MDASequence:
    return useq.MDASequence(
        channels=[useq.Channel(config=p, group=LASER_GROUP) for p in presets]
    )


def test_channels_for_sequence_selects_by_laser() -> None:
    channels = _channels()
    cams = set(CAMERAS)

    # A single laser preset saves only its matching region.
    got = channels_for_sequence(
        _sequence(["561nm"]), channels, LASER_GROUP, ALL_LASERS, cams
    )
    assert [c.name for c in got] == ["mScarlet_561"]

    # The all-lasers preset saves every ready region.
    got = channels_for_sequence(
        _sequence([ALL_LASERS]), channels, LASER_GROUP, ALL_LASERS, cams
    )
    assert len(got) == 4

    # A laser whose camera isn't present is excluded.
    got = channels_for_sequence(
        _sequence(["561nm"]), channels, LASER_GROUP, ALL_LASERS, {"Camera-1"}
    )
    assert got == []

    # No configured region matches -> nothing saved (no "save everything").
    got = channels_for_sequence(
        _sequence(["999nm"]), channels, LASER_GROUP, ALL_LASERS, cams
    )
    assert got == []


def test_channels_for_sequence_skips_regions_without_rect() -> None:
    channels = _channels()
    channels[0] = channels[0].model_copy(update={"rect": None})  # GFP_488 / 488nm
    got = channels_for_sequence(
        _sequence(["488nm"]), channels, LASER_GROUP, ALL_LASERS, set(CAMERAS)
    )
    assert got == []


def test_next_fft_friendly() -> None:
    assert next_fft_friendly(0) == 1
    assert next_fft_friendly(1) == 1
    assert next_fft_friendly(512) == 512  # 2**9, already 7-smooth
    assert next_fft_friendly(500) == 500  # 2**2 * 5**3, already 7-smooth
    assert next_fft_friendly(513) == 525  # 3**3 * 19 -> next 7-smooth is 3*5*5*7
