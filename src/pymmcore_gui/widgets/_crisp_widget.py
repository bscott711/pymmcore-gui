"""Host widget that embeds the CRISPy ASI CRISP autofocus plugin.

CRISPy's :func:`CRISPy.plugin.launch_crisp_plugin` discovers the loaded ASI CRISP
devices *at call time* and returns a fully built QWidget (a placeholder label when
no devices are found, a single panel for one device, or a tabbed panel for many).

Because the GUI caches widget-action widgets as singletons (created the first time
the action is toggled), a bare call would freeze whatever devices happened to be
loaded at that moment.  This wrapper re-runs discovery and rebuilds its child
whenever a new system configuration is loaded, so the panel always reflects the
current hardware.

It also gives CRISPy two hooks for the images its Focus curve can take at each
position: how to snap (:func:`CRISPy.imaging.set_snap_provider`), the same way the
toolbar's Snap does (:func:`pymmcore_gui.actions.core_actions.snap_image`), firing the
selected laser(s) and, on the ASI rig, going through the camera-worker pool, since the
main process's core has no cameras there; and how to show them
(:func:`CRISPy.imaging.set_image_view_factory`), with the Preview windows' own widget.
"""

from __future__ import annotations

from contextlib import suppress
from functools import partial
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus
from PyQt6.QtWidgets import QVBoxLayout, QWidget

if TYPE_CHECKING:
    import numpy as np
    from PyQt6.QtCore import QObject


def snap_for_crispy(mmc: CMMCorePlus) -> dict[str, np.ndarray]:
    """Snap one image per camera the way the toolbar's Snap does, for CRISPy.

    Called by CRISPy on the GUI thread.
    """
    from CRISPy.imaging import core_snap

    from pymmcore_gui.asi_z_stack.asi_controller import (
        close_selected_lasers,
        open_selected_lasers,
    )
    from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

    svc = CameraWorkerService.get_active()
    if svc is None and mmc.isSequenceRunning():
        mmc.stopSequenceAcquisition()
    open_selected_lasers()
    try:
        if svc is not None:
            # Camera-1/Camera-2 live in worker processes; svc.snap() stops Live.
            return svc.snap()
        return core_snap(mmc)
    finally:
        close_selected_lasers()


def crispy_image_view(mmc: CMMCorePlus, camera_label: str) -> QWidget:
    """The Preview windows' widget for *camera_label*, for CRISPy's Focus curve images.

    In its per-camera (managed) mode, as in the Preview docks: it ignores Snap and Live
    and shows only what CRISPy hands it with ``append()``.
    """
    from pymmcore_gui.widgets.image_preview._pygfx_preview import PygfxPreview

    return PygfxPreview(mmcore=mmc, camera_label=camera_label)


class CrispWidget(QWidget):
    """Container that (re)builds the CRISPy panel on system configuration load."""

    def __init__(
        self,
        parent: QWidget | None = None,
        *,
        mmcore: CMMCorePlus | None = None,
    ) -> None:
        super().__init__(parent)
        self._mmc = mmcore or CMMCorePlus.instance()

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._inner: QWidget | None = None

        self._mmc.events.systemConfigurationLoaded.connect(self._rebuild)
        self.destroyed.connect(self._disconnect)
        self._set_crispy_hooks(
            partial(snap_for_crispy, self._mmc), partial(crispy_image_view, self._mmc)
        )

        self._rebuild()

    @staticmethod
    def _set_crispy_hooks(snap: object, view: object) -> None:
        # Older CRISPy versions lack one or both hooks; skip what isn't there.
        with suppress(ImportError):
            from CRISPy import imaging

            for name, hook in (
                ("set_snap_provider", snap),
                ("set_image_view_factory", view),
            ):
                if (setter := getattr(imaging, name, None)) is not None:
                    setter(hook)

    def _disconnect(self, _obj: QObject | None = None) -> None:
        with suppress(Exception):
            self._mmc.events.systemConfigurationLoaded.disconnect(self._rebuild)
        self._set_crispy_hooks(None, None)

    def _rebuild(self) -> None:
        """Discard the current panel and rebuild it from the active configuration."""
        from CRISPy.plugin import launch_crisp_plugin

        if self._inner is not None:
            self._layout.removeWidget(self._inner)
            self._inner.deleteLater()
            self._inner = None

        inner = launch_crisp_plugin()
        inner.setParent(self)
        self._layout.addWidget(inner)
        self._inner = inner
