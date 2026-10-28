"""Tests for the image-based focus lock's engine side (no hardware).

``image_focus.py`` (the numpy logic) is covered by ``test_image_focus.py``; here
the lock runs inside the real :class:`ASISPIMEngine` ``exec_event`` loop against
a mocked core whose piezo is a tiny simulator and a mocked camera worker pool
that serves synthetic frames. The synthetic content is a Gaussian band along the
camera rows, displaced by ``ROWS_PER_UM * (piezo - focus_um)``, i.e. the same
rigid shift the lock is built on.
"""

from __future__ import annotations

import csv
import logging
from types import SimpleNamespace
from typing import TYPE_CHECKING, cast
from unittest.mock import MagicMock

import numpy as np
import pytest
import useq
from useq._mda_event import Channel as EventChannel

from pymmcore_gui._settings import (
    CalibrationRecord,
    FocusBoxSettings,
    ImageFocusSettingsV1,
    SettingsV1,
    SpectralChannelConfig,
    SpectralChannelSettingsV1,
)
from pymmcore_gui.asi_z_stack import image_focus_run as run_module
from pymmcore_gui.asi_z_stack.engine import (
    _CRISP_LOCKED_STATES,
    ASISPIMEngine,
    _ASITriggerEngineBase,
)
from pymmcore_gui.asi_z_stack.image_focus import FocusBox
from pymmcore_gui.asi_z_stack.image_focus_events import (
    CALIBRATION_METADATA_KEY,
    focus_lock_signals,
)
from pymmcore_gui.asi_z_stack.image_focus_run import (
    ImageFocusRun,
    build_calibration_sequence,
    window_for_box,
)
from pymmcore_gui.asi_z_stack.worker_pool import WorkerDiedError
from test_asi_zstack_engine import _make_engine, _setup_spim_engine

if TYPE_CHECKING:
    from collections.abc import Iterator

PIEZO = "PiezoStage:P:34"
CRISP = "CRISPAFocus:P:34"
GALVO = "Scanner:AB:33"
ROWS, COLS = 80, 40
ROWS_PER_UM = 4.0
KEY = "cam0|488nm|"
RECT = (0, 0, COLS, ROWS)


# --------------------------------------------------------------------------
# Fixtures and fakes
# --------------------------------------------------------------------------


class _Rig:
    """A piezo, a CRISP and the content they move, behind a mocked core."""

    def __init__(self, core: MagicMock, z: float = 50.0, default: str = "0.0") -> None:
        self.z = z
        self.focus_um = z  # piezo value at which the content sits on the reference
        self.crisp = "Ready"
        self.unlock_works = True
        self.moves: list[float] = []
        self.default = default
        core.getLoadedDevices.return_value = [PIEZO, CRISP, GALVO]
        core.getPosition.side_effect = lambda _label: self.z
        core.setPosition.side_effect = self._set_position
        core.getProperty.side_effect = self._get_property
        core.setProperty.side_effect = self._set_property
        core.hasProperty.return_value = True  # limits are False below
        core.hasPropertyLimits.return_value = False
        core.getXYStageDevice.return_value = ""
        self.core = core

    def _set_position(self, _label: str, z: float) -> None:
        self.moves.append(z)
        self.z = z

    def _get_property(self, _dev: str, prop: str) -> str:
        return self.crisp if prop == "CRISP State" else self.default

    def _set_property(self, _dev: str, prop: str, value: str) -> None:
        if prop == "CRISP State" and self.unlock_works:
            self.crisp = value

    @property
    def shift_rows(self) -> float:
        return ROWS_PER_UM * (self.z - self.focus_um)

    def frame(self, shift_rows: float | None = None) -> np.ndarray:
        shift = self.shift_rows if shift_rows is None else shift_rows
        rows = np.arange(ROWS, dtype=float)[:, None]
        band = 1000.0 * np.exp(-0.5 * ((rows - (40.0 + shift)) / 3.0) ** 2)
        return (band * np.ones((1, COLS)) + 100.0).astype(np.uint16)


def _image_settings(
    *, calibrated: bool = True, box: bool = True, **kw: object
) -> ImageFocusSettingsV1:
    record = CalibrationRecord(
        rows_per_um=ROWS_PER_UM,
        r2=0.99,
        range_um=(-1.5, 1.5),
        n_points=7,
        date="2026-10-26T00:00:00",
    )
    # ``mode`` is narrowed to "spot" in the settings; model_construct skips that
    # validation so the (UI-unreachable) image lock keeps its coverage.
    return ImageFocusSettingsV1.model_construct(
        mode="image",
        box=FocusBoxSettings(camera="cam0", preset="488nm", slice_index=0, rect=RECT)
        if box
        else None,
        calibrations={KEY: record} if calibrated else {},
        **kw,
    )


def _event(t: int, preset: str = "488nm") -> useq.MDAEvent:
    return useq.MDAEvent(
        index={"t": t, "c": 0},
        channel=EventChannel(config=preset, group="Lasers"),
    )


@pytest.fixture
def signals() -> Iterator[SimpleNamespace]:
    got = SimpleNamespace(reports=[], stopped=[], calibrated=[])

    def on_report(report: object) -> None:
        got.reports.append(report)

    def on_stopped(reason: str) -> None:
        got.stopped.append(reason)

    def on_calibrated(key: str, fit: object) -> None:
        got.calibrated.append((key, fit))

    focus_lock_signals.report.connect(on_report)
    focus_lock_signals.stopped.connect(on_stopped)
    focus_lock_signals.calibrated.connect(on_calibrated)
    yield got
    focus_lock_signals.report.disconnect(on_report)
    focus_lock_signals.stopped.disconnect(on_stopped)
    focus_lock_signals.calibrated.disconnect(on_calibrated)


@pytest.fixture(autouse=True)
def _no_real_roi_or_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        run_module, "_camera_roi", lambda _mmc, _cam: (0, 0, COLS, ROWS)
    )
    monkeypatch.setattr(run_module.time, "sleep", lambda _s: None)


def _rig_engine(
    cfg: ImageFocusSettingsV1 | None = None,
    sequence: useq.MDASequence | None = None,
    *,
    crisp: str = "Ready",
    start: bool = True,
) -> tuple[MagicMock, ASISPIMEngine, MagicMock, _Rig, ImageFocusRun]:
    """Engine + simulated rig + a focus run wired the way setup_sequence wires it."""
    core, engine, pool = _make_engine(n_cameras=1, n_slices=1)
    rig = _Rig(core)
    rig.crisp = crisp
    # The mock pool serves one box-sized frame per volume, from the piezo's
    # position at the moment the volume runs.
    pool.iter_frames.side_effect = lambda **_kw: iter([("cam0", 0, rig.frame(), {}, 0)])
    engine._snapshot_piezo_position()
    seq = sequence or useq.MDASequence(
        channels=[useq.Channel(config="488nm", group="Lasers", exposure=10.0)]
    )
    run = ImageFocusRun.for_sequence(
        core,
        engine.hw,
        seq,
        crisp_locked=lambda: engine._crisp_state() in _CRISP_LOCKED_STATES,
        image_focus=cfg if cfg is not None else _image_settings(),
        spectral=SpectralChannelSettingsV1(),
    )
    engine._focus_run = run
    if start:
        run.start()
    return core, cast("ASISPIMEngine", engine), pool, rig, run


def _volume(engine: _ASITriggerEngineBase, t: int, preset: str = "488nm") -> None:
    list(engine.exec_event(_event(t, preset)))


# --------------------------------------------------------------------------
# Inactive
# --------------------------------------------------------------------------


@pytest.mark.parametrize("mode", ["crisp", "off"])
def test_inactive_modes_are_a_no_op(
    mode: str, signals: SimpleNamespace, monkeypatch: pytest.MonkeyPatch
) -> None:
    """CRISP/off selected: no piezo, CRISP or signal activity at all."""
    cfg = _image_settings()
    cfg.mode = mode  # type: ignore[assignment]
    core, engine, _pool, rig, run = _rig_engine(cfg, crisp="In Focus")
    row_profile = MagicMock()
    monkeypatch.setattr(run_module, "row_profile", row_profile)

    assert run.mode == "off"
    assert not run.active
    for t in range(3):
        _volume(engine, t)

    row_profile.assert_not_called()
    assert rig.moves == []
    assert rig.crisp == "In Focus"  # not unlocked either
    core.setProperty.assert_any_call(GALVO, "SPIMState", "Running")  # volumes ran
    assert all(c.args[1] != "CRISP State" for c in core.setProperty.call_args_list)
    assert signals.reports == [] and signals.stopped == []
    run.teardown()
    assert signals.stopped == []


# --------------------------------------------------------------------------
# Image mode
# --------------------------------------------------------------------------


def test_image_mode_reference_then_correction_with_right_sign(
    signals: SimpleNamespace, caplog: pytest.LogCaptureFixture
) -> None:
    """Volume 0 is the reference; a later drift is cancelled by a piezo move."""
    _core, engine, _pool, rig, run = _rig_engine()
    assert run.active

    _volume(engine, 0)
    assert rig.moves == []
    assert signals.reports[-1].message == "reference captured"
    assert signals.reports[-1].decision is None

    # The sample drifts: in focus now needs piezo 50.5, so content sits at
    # ROWS_PER_UM * (50 - 50.5) = -2 rows. The lock must move the piezo UP, by
    # gain 0.5 * 2 rows / 4 rows/um = 0.25 um.
    rig.focus_um = 50.5
    with caplog.at_level(logging.INFO):
        _volume(engine, 1)

    assert rig.moves == [pytest.approx(50.25, abs=0.01)]
    report = signals.reports[-1]
    assert report.mode == "image"
    assert report.index == {"t": 1, "c": 0}
    assert report.piezo_um == pytest.approx(50.25, abs=0.01)
    assert report.decision is not None
    assert report.decision.error_rows == pytest.approx(-2.0, abs=0.3)
    assert run.intentional_um == pytest.approx(0.25, abs=0.01)
    # per-volume record gets the outcome and a second log line
    record = engine._volume_records[-1]
    assert record.focus_move_um == pytest.approx(0.25, abs=0.01)
    assert record.piezo_after == pytest.approx(50.0)  # before the correction
    assert any("image focus lock" in r.message for r in caplog.records)

    # Closed loop: keeps converging on 50.5 and settles inside the deadband.
    for t in range(2, 12):
        _volume(engine, t)
    assert rig.z == pytest.approx(50.5, abs=0.15)
    assert not run.last_row or abs(run.last_row.move_um or 0.0) < 0.1


def test_image_mode_is_a_noop_for_another_channels_volume(
    signals: SimpleNamespace,
) -> None:
    """A volume of a different laser is not a missed frame."""
    _core, engine, pool, rig, run = _rig_engine()
    pool.iter_frames.side_effect = lambda **_kw: iter([("cam0", 0, rig.frame(), {}, 0)])
    for t in range(5):
        _volume(engine, t, preset="561nm")
    assert run.active
    assert signals.reports == [] and signals.stopped == []


def test_missing_box_frames_hold_then_stop(signals: SimpleNamespace) -> None:
    """Volumes with no frame for the box hold; max_strikes in a row ends the lock."""
    _core, engine, pool, rig, run = _rig_engine()
    _volume(engine, 0)  # reference
    rig.focus_um = 50.5
    pool.iter_frames.side_effect = lambda **_kw: iter(
        [("cam0", 3, rig.frame(), {}, 0)]  # a slice the box does not watch
    )
    for t in (1, 2):
        _volume(engine, t)
        assert run.active
        assert "no frame for box" in signals.reports[-1].message
    _volume(engine, 3)

    assert not run.active
    assert signals.stopped and "no frame for the box" in signals.stopped[0]
    assert rig.moves == []


def test_no_after_volume_on_cancel_or_error() -> None:
    """A cancelled or failed volume never reaches the focus run's after_volume."""
    _core, engine, pool, rig, run = _rig_engine()
    after = MagicMock()
    run.after_volume = after  # type: ignore[method-assign]
    pool.iter_frames.side_effect = lambda **_kw: iter(
        [("cam0", i, rig.frame(), {}, 0) for i in range(3)]
    )

    gen = cast("Iterator", engine.exec_event(_event(0)))
    next(gen)
    with pytest.raises(StopIteration):
        gen.send("cancel")
    after.assert_not_called()

    pool.iter_frames.side_effect = WorkerDiedError("cam0", -1)
    with pytest.raises(WorkerDiedError):
        _volume(engine, 1)
    after.assert_not_called()

    # ...and a normal completion does call it.
    pool.iter_frames.side_effect = lambda **_kw: iter([("cam0", 0, rig.frame(), {}, 0)])
    _volume(engine, 2)
    after.assert_called_once_with({"t": 2, "c": 0}, "488nm")


def test_start_unlocks_crisp(
    signals: SimpleNamespace, caplog: pytest.LogCaptureFixture
) -> None:
    """A locked CRISP is put to Ready at start (INFO), never locked again."""
    with caplog.at_level(logging.INFO):
        _core, _engine, _pool, rig, run = _rig_engine(crisp="In Focus")
    assert rig.crisp == "Ready"
    assert run.active
    assert any("unlocked it" in r.message for r in caplog.records)


def test_start_gives_up_if_crisp_will_not_unlock(signals: SimpleNamespace) -> None:
    _core, engine, _pool, rig, run = _rig_engine(start=False, crisp="Lock")
    rig.unlock_works = False
    run.start()
    assert not run.active
    assert signals.stopped and "did not unlock" in signals.stopped[0]
    _volume(engine, 0)
    assert rig.moves == []


def test_crisp_locked_at_move_time_refuses(
    signals: SimpleNamespace, caplog: pytest.LogCaptureFixture
) -> None:
    """If CRISP locked again after start, the move is refused and the lock ends."""
    _core, engine, _pool, rig, run = _rig_engine()
    _volume(engine, 0)
    rig.focus_um = 50.5
    rig.crisp = "Lock"

    with caplog.at_level(logging.WARNING):
        _volume(engine, 1)

    assert rig.moves == []
    assert not run.active
    assert signals.stopped and "CRISP locked" in signals.stopped[0]
    assert run.intentional_um == 0.0
    # later drift is left alone, and teardown does not try to restore either
    _volume(engine, 2)
    run.teardown()
    assert rig.moves == []


def test_uncalibrated_or_boxless_runs_without_the_lock(
    signals: SimpleNamespace, caplog: pytest.LogCaptureFixture
) -> None:
    """No calibration / no box: WARNING + stopped, but the acquisition proceeds."""
    for cfg, text in (
        (_image_settings(calibrated=False), "not calibrated"),
        (_image_settings(box=False), "no focus box"),
    ):
        signals.stopped.clear()
        caplog.clear()
        with caplog.at_level(logging.WARNING):
            _core, engine, _pool, rig, run = _rig_engine(cfg)
        assert not run.active
        assert any(text in r.message for r in caplog.records)
        assert any(
            r.levelno == logging.WARNING and "Image focus lock" in r.message
            for r in caplog.records
        )
        assert signals.stopped and text in signals.stopped[0]
        payloads = list(engine.exec_event(_event(0)))
        assert len(payloads) == 1  # the acquisition itself is untouched
        assert rig.moves == []


def test_fall_back_stops_moving(signals: SimpleNamespace) -> None:
    """Unusable measurements fall back for good: stopped is emitted, no more moves."""
    _core, engine, pool, rig, run = _rig_engine()
    _volume(engine, 0)
    flat = np.full((ROWS, COLS), 100, dtype=np.uint16)
    pool.iter_frames.side_effect = lambda **_kw: iter([("cam0", 0, flat, {}, 0)])
    for t in (1, 2, 3):
        _volume(engine, t)

    assert not run.active
    assert signals.stopped and "rejected measurements" in signals.stopped[0]
    assert rig.moves == []

    # even a clear, correctable drift is now left alone
    rig.focus_um = 51.0
    pool.iter_frames.side_effect = lambda **_kw: iter([("cam0", 0, rig.frame(), {}, 0)])
    _volume(engine, 4)
    assert rig.moves == []


def test_piezo_moved_warning_allows_for_intentional_moves(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """The lock's own corrections are expected; motion beyond them still warns."""
    _core, engine, _pool, rig, run = _rig_engine()
    _volume(engine, 0)
    rig.focus_um = 50.5
    _volume(engine, 1)
    assert run.intentional_um == pytest.approx(0.25, abs=0.01)

    caplog.clear()
    with caplog.at_level(logging.WARNING):
        engine._warn_if_piezo_moved()
    assert not caplog.records

    rig.z += 0.5  # something else moved it
    with caplog.at_level(logging.WARNING):
        engine._warn_if_piezo_moved()
    assert len(caplog.records) == 1
    message = caplog.records[0].message
    assert "+0.500" in message
    assert "beyond the image focus lock's own +0.250" in message


@pytest.mark.parametrize(("end_of_run", "final"), [("restore", 50.0), ("stay", 50.25)])
def test_teardown_restore_or_stay(
    end_of_run: str, final: float, signals: SimpleNamespace
) -> None:
    _core, engine, _pool, rig, run = _rig_engine(_image_settings(end_of_run=end_of_run))
    _volume(engine, 0)
    rig.focus_um = 50.5
    _volume(engine, 1)
    assert rig.z == pytest.approx(50.25, abs=0.01)

    run.teardown()

    assert rig.z == pytest.approx(final, abs=0.01)
    assert signals.stopped[-1] == "run finished"


def test_csv_log_written_next_to_the_data(tmp_path, signals: SimpleNamespace) -> None:
    seq = useq.MDASequence(
        channels=[useq.Channel(config="488nm", group="Lasers", exposure=10.0)],
        metadata={
            "pymmcore_widgets": {
                "save_dir": str(tmp_path),
                "save_name": "exp1.ome.zarr",
            }
        },
    )
    _core, engine, _pool, rig, run = _rig_engine(sequence=seq)
    _volume(engine, 0)
    rig.focus_um = 50.5
    _volume(engine, 1)
    run.teardown()

    with (tmp_path / "exp1_focus_log.csv").open() as fh:
        rows = list(csv.DictReader(fh))
    assert list(rows[0]) == [
        "t",
        "c",
        "piezo_before",
        "error_rows",
        "error_um",
        "move_um",
        "confidence",
        "reason",
    ]
    assert [r["t"] for r in rows] == ["0", "1"]
    assert rows[0]["reason"] == "reference captured"
    assert float(rows[1]["move_um"]) == pytest.approx(0.25, abs=0.01)


def test_no_save_target_means_no_csv_and_no_error(tmp_path) -> None:
    _core, engine, _pool, _rig, run = _rig_engine()
    _volume(engine, 0)
    run.teardown()  # must not raise
    # Only the CSV matters: the real conftest's _mock_pyconify puts an
    # ``icons`` folder in every tmp_path.
    assert not list(tmp_path.rglob("*.csv"))


# --------------------------------------------------------------------------
# Failures never reach the acquisition
# --------------------------------------------------------------------------


def test_tap_exception_does_not_propagate(
    signals: SimpleNamespace,
    caplog: pytest.LogCaptureFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _core, engine, _pool, rig, run = _rig_engine()
    monkeypatch.setattr(
        run_module, "row_profile", MagicMock(side_effect=RuntimeError("x"))
    )

    with caplog.at_level(logging.ERROR):
        payloads = list(engine.exec_event(_event(0)))  # frames still delivered
    assert len(payloads) == 1
    assert not run.active
    assert len(signals.stopped) == 1
    assert sum("could not read the focus box" in r.message for r in caplog.records) == 1
    # a second volume: silent, no repeated logging or signalling
    _volume(engine, 1)
    assert len(signals.stopped) == 1
    assert rig.moves == []


def test_after_volume_exception_is_contained(
    signals: SimpleNamespace, caplog: pytest.LogCaptureFixture
) -> None:
    _core, engine, _pool, _rig, run = _rig_engine()
    _volume(engine, 0)
    assert run._controller is not None
    run._controller.update = MagicMock(side_effect=RuntimeError("boom"))  # type: ignore[method-assign]

    with caplog.at_level(logging.ERROR):
        _volume(engine, 1)  # does not raise

    assert not run.active
    assert signals.stopped and "boom" in signals.stopped[0]
    assert any(r.levelno == logging.ERROR for r in caplog.records)


def test_frame_geometry_mismatch_deactivates(signals: SimpleNamespace) -> None:
    """Frames not matching the camera's reported ROI would misplace the box."""
    _core, engine, pool, _rig, run = _rig_engine()
    pool.iter_frames.side_effect = lambda **_kw: iter(
        [("cam0", 0, np.zeros((ROWS, COLS + 10), dtype=np.uint16), {}, 0)]
    )
    _volume(engine, 0)
    assert not run.active
    assert signals.stopped and "ROI" in signals.stopped[0]


# --------------------------------------------------------------------------
# setup_sequence wiring
# --------------------------------------------------------------------------


def _patch_live_settings(
    monkeypatch: pytest.MonkeyPatch, cfg: ImageFocusSettingsV1
) -> None:
    live = SettingsV1.instance().image_focus
    for name in ("mode", "box", "calibrations", "end_of_run"):
        monkeypatch.setattr(live, name, getattr(cfg, name))


def test_setup_sequence_builds_and_starts_the_run(
    monkeypatch: pytest.MonkeyPatch, signals: SimpleNamespace
) -> None:
    core, engine = _setup_spim_engine(monkeypatch)
    rig = _Rig(core, default="0.1234")
    rig.crisp = "In Focus"
    _patch_live_settings(monkeypatch, _image_settings())
    seq = useq.MDASequence(
        channels=[useq.Channel(config="488nm", group="Lasers", exposure=10.0)]
    )

    engine.setup_sequence(seq)

    run = engine._focus_run
    assert run is not None and run.mode == "image" and run.active
    assert rig.crisp == "Ready"  # the lock owns the piezo now
    assert run._start_um == pytest.approx(50.0)


def test_setup_sequence_without_calibration_still_sets_up(
    monkeypatch: pytest.MonkeyPatch, signals: SimpleNamespace
) -> None:
    core, engine = _setup_spim_engine(monkeypatch)
    _Rig(core, default="0.1234")
    _patch_live_settings(monkeypatch, _image_settings(calibrated=False))

    engine.setup_sequence(
        useq.MDASequence(channels=[useq.Channel(config="488nm", group="Lasers")])
    )

    assert engine._focus_run is not None and not engine._focus_run.active
    assert signals.stopped and "not calibrated" in signals.stopped[0]


def test_teardown_sequence_restores_before_the_piezo_check(
    monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    core, engine = _setup_spim_engine(monkeypatch)
    rig = _Rig(core, default="0.1234")
    monkeypatch.setattr(run_module.time, "sleep", lambda _s: None)
    monkeypatch.setattr(engine, "_reclaim_from_workers", MagicMock())
    monkeypatch.setattr("pymmcore_gui.asi_z_stack.engine.time.sleep", lambda _s: None)
    _patch_live_settings(monkeypatch, _image_settings())
    seq = useq.MDASequence(
        channels=[useq.Channel(config="488nm", group="Lasers", exposure=10.0)]
    )
    engine.setup_sequence(seq)
    run = engine._focus_run
    assert run is not None
    # pretend the lock corrected the piezo during the run
    rig.z = 50.25
    run.intentional_um = 0.25

    with caplog.at_level(logging.WARNING):
        engine.teardown_sequence(seq)

    assert rig.z == pytest.approx(50.0)  # restored
    assert not [r for r in caplog.records if "moved" in r.message]  # and no warning
    assert engine._focus_run is None


# --------------------------------------------------------------------------
# Calibration
# --------------------------------------------------------------------------

BOX = FocusBox("cam0", "488nm", 5, RECT)


def _base_sequence(**kw: object) -> useq.MDASequence:
    return useq.MDASequence(
        channels=[
            useq.Channel(config="561nm", group="Lasers", exposure=30.0),
            useq.Channel(config="488nm", group="Lasers", exposure=20.0),
        ],
        z_plan=kw.pop("z_plan", useq.ZRangeAround(range=10, step=1)),
        time_plan=useq.TIntervalLoops(interval=60, loops=100),
        stage_positions=[(1.0, 2.0), (3.0, 4.0)],
        metadata={"pymmcore_widgets": {"save_dir": "S:/x", "save_name": "run"}},
        **kw,
    )


def test_calibration_sequence_shape() -> None:
    base = _base_sequence()
    seq = build_calibration_sequence(base, BOX, window="mNG_488nm")

    assert [c.config for c in seq.channels] == ["488nm"]
    assert seq.channels[0].exposure == 20.0  # taken from the base channel
    # one slice, at the plane slice 5 of ZRangeAround(10, 1) images (z = 0)
    assert list(seq.z_plan) == [pytest.approx(0.0)]
    assert seq.z_plan.is_relative
    # 7 steps x 2 reps, back to back
    assert seq.time_plan.loops == 14 and seq.time_plan.interval.total_seconds() == 0
    assert len(list(seq)) == 14
    assert {e.index["z"] for e in seq} == {0}
    assert len(seq.stage_positions) == 1 and seq.stage_positions[0].x == 1.0
    payload = seq.metadata[CALIBRATION_METADATA_KEY]
    assert payload == {
        "steps_um": [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5],
        "reps": 2,
        "box": {
            "camera": "cam0",
            "preset": "488nm",
            "slice_index": 5,
            "rect": [0, 0, 40, 80],
        },
        "window": "mNG_488nm",
    }
    # no save name / dir: the Argus streamer (which needs one) and the writers skip it
    assert "pymmcore_widgets" not in seq.metadata


@pytest.mark.parametrize(("slice_index", "expected"), [(0, -5.0), (3, -2.0), (10, 5.0)])
def test_calibration_sequence_picks_the_boxs_plane(
    slice_index: int, expected: float
) -> None:
    box = FocusBox("cam0", "488nm", slice_index, RECT)
    seq = build_calibration_sequence(_base_sequence(), box)
    assert list(seq.z_plan) == [pytest.approx(expected)]


def test_calibration_sequence_absolute_and_descending_plans() -> None:
    absolute = _base_sequence(z_plan=useq.ZTopBottom(top=110, bottom=100, step=1))
    seq = build_calibration_sequence(absolute, FocusBox("cam0", "488nm", 2, RECT))
    assert not seq.z_plan.is_relative and list(seq.z_plan) == [102.0]

    down = _base_sequence(z_plan=useq.ZRangeAround(range=4, step=1, go_up=False))
    seq = build_calibration_sequence(down, FocusBox("cam0", "488nm", 0, RECT))
    assert list(seq.z_plan) == [2.0]  # slice 0 of a descending plan is the top


def test_calibration_sequence_without_z_plan() -> None:
    base = useq.MDASequence(channels=[useq.Channel(config="488nm", group="Lasers")])
    seq = build_calibration_sequence(base, FocusBox("cam0", "488nm", 0, RECT))
    assert seq.z_plan is None and len(list(seq)) == 14
    with pytest.raises(ValueError, match="no z plan"):
        build_calibration_sequence(base, FocusBox("cam0", "488nm", 2, RECT))


def test_calibration_sequence_errors_are_clear() -> None:
    with pytest.raises(ValueError, match="slice 11"):
        build_calibration_sequence(
            _base_sequence(), FocusBox("cam0", "488nm", 11, RECT)
        )
    with pytest.raises(ValueError, match="no channel with preset '405nm'"):
        build_calibration_sequence(_base_sequence(), FocusBox("cam0", "405nm", 0, RECT))
    with pytest.raises(ValueError, match="at least 3"):
        build_calibration_sequence(_base_sequence(), BOX, steps_um=(0.0, 1.0))


@pytest.mark.parametrize(
    ("z_plan", "focus", "offset_um"),
    [
        (useq.ZRangeAround(range=10, step=1), 0.0, 0.0),
        (useq.ZAboveBelow(above=10, below=0, step=1), 0.0, 5.0),
        (useq.ZTopBottom(top=110, bottom=100, step=1), 100.0, 5.0),
    ],
)
def test_engine_copes_with_the_one_slice_calibration_plan(
    monkeypatch: pytest.MonkeyPatch,
    z_plan: useq.ZPlan,
    focus: float,
    offset_um: float,
) -> None:
    """One slice, zero galvo amplitude, centred on the very plane slice 5 imaged."""
    from test_asi_zstack_engine import _galvo_value

    core, engine = _setup_spim_engine(monkeypatch, focus_um=focus)
    _Rig(core, z=focus, default="0.1234")
    base = _base_sequence(z_plan=z_plan)
    # the plane slice 5 images in the full stack, relative to the Live plane
    full = [float(p) for p in z_plan]
    reference = 0.0 if z_plan.is_relative else focus
    plane = full[5] - reference
    seq = build_calibration_sequence(base, BOX)

    engine.setup_sequence(seq)

    slope = engine.hw.slice_calibration_slope_um_per_deg
    assert engine._num_slices == 1
    assert _galvo_value(core, "SingleAxisYAmplitude(deg)") == pytest.approx(0.0)
    assert _galvo_value(core, "SingleAxisYOffset(deg)") == pytest.approx(
        0.1234 + plane / slope, abs=1e-4
    )
    assert engine._focus_run is not None and engine._focus_run.mode == "calibrate"


def test_calibrate_run_steps_the_piezo_fits_and_restores(
    monkeypatch: pytest.MonkeyPatch, signals: SimpleNamespace
) -> None:
    live = SettingsV1.instance()
    old = CalibrationRecord(
        rows_per_um=-1.0, r2=0.5, range_um=(0, 1), n_points=3, date="old"
    )
    monkeypatch.setitem(live.image_focus.calibrations, KEY, old)
    flush = MagicMock()
    monkeypatch.setattr(SettingsV1, "flush", flush)

    seq = build_calibration_sequence(_base_sequence(), BOX)
    # settings mode "crisp": a calibration is chosen by its metadata alone
    cfg = ImageFocusSettingsV1.model_construct(mode="crisp")
    _core, engine, pool, rig, run = _rig_engine(cfg, seq, start=False)
    assert run.mode == "calibrate" and run.active
    seen_z: list[float] = []

    def frames(**_kw: object):
        seen_z.append(rig.z)
        return iter([("cam0", 0, rig.frame(), {}, 0)])

    pool.iter_frames.side_effect = frames
    run.start()
    assert rig.z == pytest.approx(50.0 - 1.5)  # parked on the first step

    for event in engine.event_iterator(seq):
        list(engine.exec_event(event))
        engine._note_focus_outcome(None)

    steps = [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]
    assert seen_z == [pytest.approx(50.0 + s) for s in steps for _ in range(2)]

    assert len(signals.calibrated) == 1
    key, fit = signals.calibrated[0]
    assert key == KEY
    assert fit.rows_per_um == pytest.approx(ROWS_PER_UM, abs=0.1)
    assert fit.r2 > 0.99
    saved = live.image_focus.calibrations[KEY]
    assert saved.rows_per_um == pytest.approx(fit.rows_per_um)
    assert saved is not old
    flush.assert_called_once()

    # the last step is still where the piezo is until teardown restores it
    assert rig.z == pytest.approx(51.5)
    run.teardown()
    assert rig.z == pytest.approx(50.0)
    assert signals.stopped == ["run finished"]
    assert [r.mode for r in signals.reports] == ["calibrate"] * 14


def test_calibration_with_a_bad_fit_is_not_saved(
    monkeypatch: pytest.MonkeyPatch, signals: SimpleNamespace
) -> None:
    live = SettingsV1.instance()
    monkeypatch.setitem(live.image_focus.calibrations, "sentinel", MagicMock())
    monkeypatch.setattr(SettingsV1, "flush", MagicMock())
    seq = build_calibration_sequence(_base_sequence(), BOX)
    _core, engine, pool, rig, run = _rig_engine(ImageFocusSettingsV1(), seq)
    rng = np.random.default_rng(0)
    # the content jumps around at random: no linear response to the piezo
    pool.iter_frames.side_effect = lambda **_kw: iter(
        [("cam0", 0, rig.frame(rng.uniform(-8, 8)), {}, 0)]
    )
    for event in engine.event_iterator(seq):
        list(engine.exec_event(event))

    assert signals.calibrated == []
    assert KEY not in live.image_focus.calibrations
    assert signals.stopped and (
        "not saved" in signals.stopped[0] or "calibrat" in signals.stopped[0]
    )
    run.teardown()
    assert rig.z == pytest.approx(50.0)  # restored regardless


def test_calibration_refuses_to_move_while_crisp_is_locked(
    signals: SimpleNamespace,
) -> None:
    seq = build_calibration_sequence(_base_sequence(), BOX)
    _core, _engine, _pool, rig, run = _rig_engine(
        ImageFocusSettingsV1(), seq, start=False
    )
    rig.crisp = "Lock"
    rig.unlock_works = False
    run.start()
    assert not run.active and rig.moves == []
    run.teardown()
    assert rig.moves == []


# --------------------------------------------------------------------------
# Spectral window
# --------------------------------------------------------------------------


def test_window_for_box() -> None:
    spectral = SpectralChannelSettingsV1(
        enabled=True,
        channels=[
            SpectralChannelConfig(
                name="top", camera="cam0", laser_preset="488nm", rect=(0, 0, 100, 50)
            ),
            SpectralChannelConfig(
                name="bottom",
                camera="cam0",
                laser_preset="405nm",
                rect=(0, 50, 100, 50),
            ),
            SpectralChannelConfig(name="undrawn", camera="cam0", laser_preset="x"),
        ],
    )
    assert (
        window_for_box(FocusBox("cam0", "488nm", 0, (10, 10, 20, 20)), spectral)
        == "top"
    )
    assert (
        window_for_box(FocusBox("cam0", "405nm", 0, (10, 60, 20, 20)), spectral)
        == "bottom"
    )
    # straddles both windows, other camera, spectral off: no window
    assert (
        window_for_box(FocusBox("cam0", "488nm", 0, (10, 40, 20, 20)), spectral) == ""
    )
    assert (
        window_for_box(FocusBox("cam1", "488nm", 0, (10, 10, 20, 20)), spectral) == ""
    )
    spectral.enabled = False
    assert (
        window_for_box(FocusBox("cam0", "488nm", 0, (10, 10, 20, 20)), spectral) == ""
    )
    assert window_for_box(FocusBox("cam0", "488nm", 0, (10, 10, 20, 20)), None) == ""
