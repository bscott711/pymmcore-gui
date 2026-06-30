"""Host widget that embeds the CRISPy ASI CRISP autofocus plugin.

CRISPy's :func:`CRISPy.plugin.launch_crisp_plugin` discovers the loaded ASI CRISP
devices *at call time* and returns a fully built QWidget (a placeholder label when
no devices are found, a single panel for one device, or a tabbed panel for many).

Because the GUI caches widget-action widgets as singletons (created the first time
the action is toggled), a bare call would freeze whatever devices happened to be
loaded at that moment.  This wrapper re-runs discovery and rebuilds its child
whenever a new system configuration is loaded, so the panel always reflects the
current hardware.
"""

from __future__ import annotations

from contextlib import suppress
from typing import TYPE_CHECKING

from pymmcore_plus import CMMCorePlus
from PyQt6.QtWidgets import QVBoxLayout, QWidget

if TYPE_CHECKING:
    from PyQt6.QtCore import QObject


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

        self._rebuild()

    def _disconnect(self, _obj: QObject | None = None) -> None:
        with suppress(Exception):
            self._mmc.events.systemConfigurationLoaded.disconnect(self._rebuild)

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
