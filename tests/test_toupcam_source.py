"""ToupcamSource against a fake ``toupcam.py`` that keeps the real SDK's rules.

The rules that matter (SDK 20260519 docs): in RAW mode PullImage ignores its
``bits`` argument and the sample depth comes from OPTION_BITDEPTH; frames are
the ROI's size; ``put_Roi`` waits for a running callback, so calling it while
holding the lock the callback takes deadlocks; ``put_Roi`` takes coordinates
relative to the full sensor and ``(0, 0, 0, 0)`` clears the ROI; ``PullImageV2``
reports the size of the frame it wrote.
"""

from __future__ import annotations

import textwrap

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack.toupcam_source import ToupcamSource

FAKE_SDK = textwrap.dedent(
    """
    import ctypes

    TOUPCAM_EVENT_IMAGE = 0x0004
    TOUPCAM_OPTION_RAW = 0x04
    TOUPCAM_OPTION_BITDEPTH = 0x06
    FULL = (64, 48)
    EIGHT_BIT_ONLY = False  # RAW8-only models reject OPTION_BITDEPTH
    FAIL_EXPO = False

    class ToupcamFrameInfoV2:
        def __init__(self):
            self.width = 0
            self.height = 0
            self.flag = 0
            self.seq = 0
            self.timestamp = 0

    class _Dev:
        id = "tp-0001"
        displayname = "MD310C-BS"

    class Toupcam:
        last = None

        @classmethod
        def EnumV2(cls):
            return [_Dev()]

        @classmethod
        def Open(cls, camId):
            cls.last = cls()
            return cls.last

        def __init__(self):
            self.closed = 0
            self.options = {}
            self.roi = (0, 0, 0, 0)
            self.roi_calls = []
            self.cb = None
            self.lock_probe = None  # set by the test: returns True if held

        def put_Option(self, k, v):
            if k == TOUPCAM_OPTION_BITDEPTH and EIGHT_BIT_ONLY:
                raise RuntimeError("HRESULTException: Not implemented")
            self.options[k] = v

        def put_AutoExpoEnable(self, v):
            pass

        def put_ExpoTime(self, us):
            if FAIL_EXPO:
                raise RuntimeError("put_ExpoTime failed")
            self.expo_us = us

        def put_ExpoAGain(self, g):
            self.gain = g

        def get_ExpTimeRange(self):
            return (105, 15000000, 10000)  # min, max, default in microseconds

        def get_ExpoAGainRange(self):
            return (100, 5000, 100)  # min, max, default in percent

        def get_Size(self):
            return FULL

        def get_Roi(self):
            return self.roi

        def put_Roi(self, x, y, w, h):
            if self.lock_probe is not None and self.lock_probe():
                raise RuntimeError("deadlock: put_Roi called under the callback lock")
            self.roi = (x, y, w, h)
            self.roi_calls.append((x, y, w, h))

        def get_RawFormat(self):
            bits = 12 if self.options.get(TOUPCAM_OPTION_BITDEPTH) else 8
            return (0x47425247, bits)

        def StartPullModeWithCallback(self, fun, ctx):
            self.cb = (fun, ctx)

        def PullImageV2(self, buf, bits, info):
            w, h = (self.roi[2], self.roi[3]) if self.roi[2] else FULL
            deep = self.options.get(TOUPCAM_OPTION_BITDEPTH)
            n = w * h * (2 if deep else 1)
            assert len(buf) >= n, (len(buf), n)  # the SDK writes the frame's bytes
            if info is not None:
                info.width, info.height = w, h
            val = 4000 if deep else 200
            data = (val).to_bytes(2 if deep else 1, "little") * (w * h)
            ctypes.memmove(buf, data, n)

        def fire(self):
            fun, ctx = self.cb
            fun(TOUPCAM_EVENT_IMAGE, ctx)

        def Close(self):
            self.closed += 1
    """
)


@pytest.fixture
def source(tmp_path):
    (tmp_path / "toupcam.py").write_text(FAKE_SDK)
    src = ToupcamSource(sdk_dir=str(tmp_path))
    src.open()
    yield src
    src.close()


def _cam(src):
    return src._sdk.Toupcam.last


def test_raw_depth_comes_from_bitdepth_option(source):
    cam = _cam(source)
    assert cam.options[source._sdk.TOUPCAM_OPTION_RAW] == 1
    assert cam.options[source._sdk.TOUPCAM_OPTION_BITDEPTH] == 1
    cam.fire()
    frame, _t = source.latest(1.0)
    assert frame.dtype == np.uint16 and frame.shape == (48, 64)
    assert int(frame.max()) == 4000
    assert source.info()["bits"] == 12


def test_frames_take_the_roi_size(source):
    source.set_roi(10, 6, 20, 16)
    _cam(source).fire()
    frame, _t = source.latest(1.0)
    assert frame.shape == (16, 20)


def test_set_roi_does_not_hold_the_callback_lock(source):
    cam = _cam(source)
    cam.lock_probe = source._lock.locked
    source.set_roi(0, 0, 32, 32)  # the fake raises if the lock is held

    # Negative control: the old code took the lock around put_Roi.
    with source._lock, pytest.raises(RuntimeError, match="deadlock"):
        cam.put_Roi(0, 0, 32, 32)


def _make(tmp_path, **flags):
    sdk = FAKE_SDK
    for name, value in flags.items():
        sdk = sdk.replace(f"{name} = False", f"{name} = {value}", 1)
    (tmp_path / "toupcam.py").write_text(sdk)
    return ToupcamSource(sdk_dir=str(tmp_path))


def test_eight_bit_only_camera_opens(tmp_path):
    # ICMOS03100KPB: put_Option(BITDEPTH) raises "Not implemented".
    src = _make(tmp_path, EIGHT_BIT_ONLY=True)
    src.open()
    try:
        cam = _cam(src)
        assert src._sdk.TOUPCAM_OPTION_BITDEPTH not in cam.options
        cam.fire()
        frame, _t = src.latest(1.0)
        assert frame.dtype == np.uint8 and frame.shape == (48, 64)
        assert int(frame.max()) == 200
        assert src.info()["bits"] == 8
    finally:
        src.close()


def test_failed_open_closes_the_handle(tmp_path):
    src = _make(tmp_path, FAIL_EXPO=True)
    with pytest.raises(RuntimeError, match="put_ExpoTime"):
        src.open()
    assert src._cam is None
    assert _cam(src).closed == 1


def test_set_roi_zero_clears_the_roi(source):
    cam = _cam(source)
    source.set_roi(10, 6, 20, 16)
    assert source.info()["roi"] == (10, 6, 20, 16)
    source.set_roi(0, 0, 0, 0)
    assert cam.roi_calls[-1] == (0, 0, 0, 0)  # the SDK's "no ROI"
    assert source.info()["roi"] == (0, 0, 64, 48)
    cam.fire()
    frame, _t = source.latest(1.0)
    assert frame.shape == (48, 64)


def test_roi_is_even_and_clamped_to_the_sensor(source):
    cam = _cam(source)
    source.set_roi(11, 7, 21, 17)
    assert cam.roi_calls[-1] == (10, 6, 20, 16)
    source.set_roi(60, 40, 30, 30)  # hangs over the 64x48 sensor
    x, y, w, h = cam.roi_calls[-1]
    assert (w, h) == (30, 30) and x + w <= 64 and y + h <= 48
    source.set_roi(0, 0, 64, 48)  # the whole sensor is no ROI at all
    assert cam.roi_calls[-1] == (0, 0, 0, 0)


def test_info_reports_the_sensor_and_current_settings(source):
    info = source.info()
    assert info["sensor"] == (64, 48)
    assert info["roi"] == (0, 0, 64, 48)
    source.set_roi(10, 6, 20, 16)
    info = source.info()
    assert info["sensor"] == (64, 48)  # the sensor does not follow the ROI
    assert info["size"] == (20, 16)
    source.set_exposure_ms(25.0)
    source.set_gain(300)
    info = source.info()
    assert info["exposure_ms"] == 25.0 and info["gain"] == 300
    assert _cam(source).expo_us == 25000 and _cam(source).gain == 300


def test_exposure_and_gain_ranges_come_from_the_sdk(source):
    assert source.exposure_range_ms() == pytest.approx((0.105, 15000.0))
    assert source.gain_range() == (100, 5000)
    # Out-of-range requests are clamped to what the camera accepts.
    source.set_exposure_ms(1e9)
    source.set_gain(10)
    assert source.info()["exposure_ms"] == pytest.approx(15000.0)
    assert source.info()["gain"] == 100


def test_frame_size_comes_from_the_sdk_not_our_bookkeeping(source):
    """A frame pulled between put_Roi and the re-allocation (the SDK callback
    runs on its own thread) must be read at the size the SDK wrote."""
    cam = _cam(source)
    source.set_roi(10, 6, 20, 16)
    cam.roi = (0, 0, 64, 48)  # the SDK already switched; we have not noticed
    cam.fire()
    frame, _t = source.latest(1.0)
    assert frame.shape == (48, 64)


def test_exposure_range_failure_falls_back(tmp_path):
    (tmp_path / "toupcam.py").write_text(
        FAKE_SDK.replace("def get_ExpTimeRange", "def _gone_ExpTimeRange")
    )
    src = ToupcamSource(sdk_dir=str(tmp_path))
    src.open()
    try:
        lo, hi = src.exposure_range_ms()
        assert 0 < lo < hi
    finally:
        src.close()
