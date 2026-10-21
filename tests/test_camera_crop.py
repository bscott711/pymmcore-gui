from __future__ import annotations

from pymmcore_gui._camera_crop import crop_targets
from pymmcore_gui._settings import SpectralChannelConfig, SpectralChannelSettingsV1


def _spectral(enabled: bool, crop: bool) -> SpectralChannelSettingsV1:
    return SpectralChannelSettingsV1(
        enabled=enabled,
        crop_camera_roi=crop,
        channels=[
            SpectralChannelConfig(
                name="a", camera="Camera-1", laser_preset="488nm", rect=(10, 20, 30, 5)
            ),
            SpectralChannelConfig(
                name="b", camera="Camera-1", laser_preset="405nm", rect=(10, 40, 30, 5)
            ),
            SpectralChannelConfig(name="c", camera="Camera-2", laser_preset="561nm"),
        ],
    )


def test_crop_targets_bounding_box_per_camera() -> None:
    targets = crop_targets(_spectral(True, True), ["Camera-1", "Camera-2"])
    # Camera-2 has no drawn region, so it stays full frame
    assert targets == {"Camera-1": (10, 20, 30, 25), "Camera-2": None}


def test_crop_targets_full_frame_when_off() -> None:
    for enabled, crop in ((True, False), (False, True)):
        targets = crop_targets(_spectral(enabled, crop), ["Camera-1"])
        assert targets == {"Camera-1": None}
