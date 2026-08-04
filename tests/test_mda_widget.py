from __future__ import annotations

from typing import TYPE_CHECKING, Any
from unittest.mock import patch

import numpy as np
import pytest
import useq

from pymmcore_gui._spectral_channel_handler import _strip_known_suffix
from pymmcore_gui.widgets._mda_widget import GuiMDAWidget

if TYPE_CHECKING:
    from pathlib import Path
    from threading import Thread

    from pymmcore_plus import CMMCorePlus
    from pytestqt.qtbot import QtBot


def _spectral_split(base: str | Path, sequence: useq.MDASequence) -> list[str]:
    """Emulate a spectral split into a single wavelength-named channel file.

    Mirrors ``channel_output_path`` for a channel named ``CF647_638`` (638 nm),
    whose trailing ``_638`` the base uniquifier would misread as a counter.
    """
    stem = _strip_known_suffix(str(base))
    extension = str(base)[len(stem) :]
    return [f"{stem}_CF647_638{extension}"]


def test_next_available_path_ignores_channel_wavelength(
    mmcore: CMMCorePlus, qtbot: QtBot, tmp_path: Path
) -> None:
    """A per-channel wavelength suffix must not be treated as a counter.

    Regression test: ``exp.ome.zarr`` saved via the spectral handler writes
    ``exp_CF647_638.ome.zarr``; the base widget's uniquifier read the ``_638``
    as a counter and bumped the name to ``exp_639.ome.zarr``.
    """
    wdg = GuiMDAWidget(mmcore=mmcore)
    qtbot.addWidget(wdg)
    requested = tmp_path / "exp.ome.zarr"

    with patch.object(wdg, "_split_output_paths", _spectral_split):
        # nothing written yet -> keep the name the user typed
        assert wdg.get_next_available_path(requested) == requested

        # a first acquisition eagerly created the 638 nm channel's zarr dir
        (tmp_path / "exp_CF647_638.ome.zarr").mkdir()

        # the next path advances a clean generation counter, NOT to exp_639
        assert wdg.get_next_available_path(requested) == tmp_path / "exp_001.ome.zarr"

        # once that generation is on disk too, advance again (no counter stacking)
        (tmp_path / "exp_001_CF647_638.ome.zarr").mkdir()
        assert wdg.get_next_available_path(requested) == tmp_path / "exp_002.ome.zarr"


def test_next_available_path_non_split_uses_base_behavior(
    mmcore: CMMCorePlus, qtbot: QtBot, tmp_path: Path
) -> None:
    """When output isn't split, defer to the base widget's uniquifier."""
    wdg = GuiMDAWidget(mmcore=mmcore)
    qtbot.addWidget(wdg)
    requested = tmp_path / "exp.ome.zarr"

    with patch.object(wdg, "_split_output_paths", lambda base, seq: []):
        assert wdg.get_next_available_path(requested) == requested
        (tmp_path / "exp.ome.zarr").mkdir()
        assert wdg.get_next_available_path(requested) == tmp_path / "exp_001.ome.zarr"


def test_single_camera_save_avoids_tensorstore(
    mmcore: CMMCorePlus, qtbot: QtBot, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Single-camera, non-spectral saves route through the vendored writer.

    Regression test for the disk-save half of the tensorstore-crash fix: a
    bare str/Path output used to fall through to pymmcore-plus's
    ``OmeWritersSink`` -> ``ome_writers`` -> its (native) tensorstore
    backend. It's now routed through the same vendored, tensorstore-free
    ``handler_for_path`` dispatcher the multi-camera/spectral paths already
    use, and lands as a plain, re-openable OME-Zarr (v2) directory written by
    the vendored ``OMEZarrWriter``.
    """
    ts = pytest.importorskip("tensorstore")
    zarr = pytest.importorskip("zarr")

    def _boom(*args: object, **kwargs: object) -> None:
        raise AssertionError("single-camera save must not create a tensorstore store")

    monkeypatch.setattr(ts, "open", _boom)

    wdg = GuiMDAWidget(mmcore=mmcore)
    qtbot.addWidget(wdg)

    sequence = useq.MDASequence(
        channels=["DAPI"],  # pyright: ignore[reportArgumentType]
        time_plan=useq.TIntervalLoops(interval=0, loops=2),  # pyright: ignore
    )
    monkeypatch.setattr(wdg, "value", lambda: sequence)

    threads: list[Thread] = []
    orig_run_mda = mmcore.run_mda

    def _capture_run_mda(*args: Any, **kwargs: Any) -> Thread:
        t = orig_run_mda(*args, **kwargs)
        threads.append(t)
        return t

    monkeypatch.setattr(mmcore, "run_mda", _capture_run_mda)

    out = tmp_path / "exp.ome.zarr"
    wdg.execute_mda(out)

    assert len(threads) == 1
    threads[0].join(5)
    assert not threads[0].is_alive()

    assert out.exists()
    group = zarr.open(str(out), mode="r")
    arr = group["p0"]
    # 2 timepoints written; trailing two dims are the frame (y, x).
    assert int(np.prod(arr.shape[:-2])) == 2
