"""The snap provider the CRISPy host widget gives CRISPy's Focus curve."""

from __future__ import annotations

from unittest.mock import MagicMock

import numpy as np
import pytest

from pymmcore_gui.asi_z_stack import asi_controller, camera_worker_service
from pymmcore_gui.widgets import _crisp_widget
from pymmcore_gui.widgets._crisp_widget import snap_for_crispy


@pytest.fixture
def lasers(monkeypatch: pytest.MonkeyPatch) -> list[str]:
    calls: list[str] = []
    monkeypatch.setattr(
        asi_controller, "open_selected_lasers", lambda: calls.append("on")
    )
    monkeypatch.setattr(
        asi_controller, "close_selected_lasers", lambda: calls.append("off")
    )
    return calls


def _service(monkeypatch: pytest.MonkeyPatch, svc: object) -> None:
    monkeypatch.setattr(
        camera_worker_service.CameraWorkerService, "get_active", lambda: svc
    )


def test_snaps_through_the_camera_worker_service(
    monkeypatch: pytest.MonkeyPatch, lasers: list[str]
) -> None:
    frames = {"Camera-1": np.ones((4, 4)), "Camera-2": np.zeros((4, 4))}
    svc = MagicMock()
    svc.snap.return_value = frames
    _service(monkeypatch, svc)
    mmc = MagicMock()
    assert snap_for_crispy(mmc) is frames
    assert lasers == ["on", "off"]
    mmc.snapImage.assert_not_called()


def test_without_the_service_snaps_with_the_core(
    monkeypatch: pytest.MonkeyPatch, lasers: list[str]
) -> None:
    _service(monkeypatch, None)
    mmc = MagicMock()
    mmc.isSequenceRunning.return_value = True
    mmc.getNumberOfCameraChannels.return_value = 1
    mmc.getCameraDevice.return_value = "Camera"
    mmc.getImage.return_value = np.ones((4, 4))
    assert list(snap_for_crispy(mmc)) == ["Camera"]
    mmc.stopSequenceAcquisition.assert_called_once()  # like the toolbar's Snap
    mmc.snapImage.assert_called_once()
    assert lasers == ["on", "off"]


def test_the_lasers_go_off_when_the_snap_fails(
    monkeypatch: pytest.MonkeyPatch, lasers: list[str]
) -> None:
    svc = MagicMock()
    svc.snap.side_effect = RuntimeError("worker timed out")
    _service(monkeypatch, svc)
    with pytest.raises(RuntimeError, match="worker timed out"):
        snap_for_crispy(MagicMock())
    assert lasers == ["on", "off"]


def test_the_widget_registers_and_clears_the_provider(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from CRISPy import imaging

    seen: list[object] = []
    monkeypatch.setattr(imaging, "set_snap_provider", seen.append)
    _crisp_widget.CrispWidget._set_snap_provider("provider")
    _crisp_widget.CrispWidget._set_snap_provider(None)
    assert seen == ["provider", None]
