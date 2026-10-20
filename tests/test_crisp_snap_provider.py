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


def test_the_widget_registers_and_clears_both_hooks(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from CRISPy import imaging

    seen: list[tuple[str, object]] = []
    monkeypatch.setattr(
        imaging, "set_snap_provider", lambda h: seen.append(("snap", h))
    )
    monkeypatch.setattr(
        imaging, "set_image_view_factory", lambda h: seen.append(("view", h))
    )
    _crisp_widget.CrispWidget._set_crispy_hooks("snapper", "viewer")
    _crisp_widget.CrispWidget._set_crispy_hooks(None, None)
    assert seen == [
        ("snap", "snapper"),
        ("view", "viewer"),
        ("snap", None),
        ("view", None),
    ]


def test_an_older_crispy_without_the_view_hook_is_fine(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from CRISPy import imaging

    seen: list[object] = []
    monkeypatch.setattr(imaging, "set_snap_provider", seen.append)
    monkeypatch.delattr(imaging, "set_image_view_factory")
    _crisp_widget.CrispWidget._set_crispy_hooks("snapper", "viewer")
    assert seen == ["snapper"]


def test_the_view_is_the_preview_windows_widget(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from pymmcore_gui.widgets.image_preview import _pygfx_preview

    made: list[dict] = []

    class FakePreview:
        def __init__(self, **kwargs: object) -> None:
            made.append(kwargs)

    monkeypatch.setattr(_pygfx_preview, "PygfxPreview", FakePreview)
    mmc = MagicMock()
    view = _crisp_widget.crispy_image_view(mmc, "Camera-2")
    assert isinstance(view, FakePreview)
    assert made == [{"mmcore": mmc, "camera_label": "Camera-2"}]  # per-camera mode
