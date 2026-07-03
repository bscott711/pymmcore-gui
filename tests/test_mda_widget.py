from __future__ import annotations

from typing import TYPE_CHECKING
from unittest.mock import patch

from pymmcore_gui._spectral_channel_handler import _strip_known_suffix
from pymmcore_gui.widgets._mda_widget import GuiMDAWidget

if TYPE_CHECKING:
    from pathlib import Path

    import useq
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
