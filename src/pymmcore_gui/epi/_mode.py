"""Scope mode (OPM vs EPI): the enum, the lookup, and the mode controller.

:func:`current_mode` is what hardware-free helpers and guards consult. The
:class:`ModeController` registers itself with :func:`set_active_controller` when
the main window creates it; until then (tests, the demo setup, plugins)
:func:`current_mode` is always ``OPM``, so they see exactly today's behaviour.

The mode always starts as OPM and is never persisted (D3). The controller
only toggles it through :func:`~pymmcore_gui.epi._tiger_guard.enter_epi` /
:func:`~pymmcore_gui.epi._tiger_guard.exit_epi`; every config load returns it to
OPM through :meth:`ModeController.force_opm`.
"""

from __future__ import annotations

import enum
import logging
import weakref
from typing import TYPE_CHECKING, Any

from pymmcore_gui._qt.QtCore import QObject, Signal

if TYPE_CHECKING:
    from collections.abc import Callable

    from pymmcore_plus import CMMCorePlus
    from pymmcore_plus.mda import MDAEngine

    from pymmcore_gui.epi._tiger_guard import TigerSnapshot

logger = logging.getLogger("pymmcore_gui")


class ScopeMode(enum.Enum):
    """Which imaging path the app is in."""

    OPM = "OPM"
    EPI = "EPI"


class EpiModeRefusal(RuntimeError):
    """An OPM-owned operation was refused because the app is in EPI mode.

    A ``RuntimeError`` so it is handled like the camera worker service's other
    refusals (busy, not ready, MDA running).
    """


_active_controller: object | None = None


def set_active_controller(ctrl: object | None) -> None:
    """Register (or clear, with ``None``) the object that owns the current mode.

    The controller must expose a ``mode`` attribute holding a `ScopeMode`.
    """
    global _active_controller
    _active_controller = ctrl


def _is_deleted(obj: object) -> bool:
    """Whether *obj* is a Qt object whose C++ side is gone (a closed window)."""
    try:
        from PyQt6 import sip

        return bool(sip.isdeleted(obj))  # type: ignore[arg-type]
    except (ImportError, TypeError):
        return False  # not a Qt wrapper (a test fake) or another Qt binding


def active_controller() -> Any | None:
    """Return the registered mode controller (duck-typed), or ``None``.

    A controller whose window was destroyed counts as none.
    """
    ctrl = _active_controller
    if ctrl is None or _is_deleted(ctrl):
        return None
    return ctrl


def current_mode() -> ScopeMode:
    """Return the current mode; ``OPM`` when no controller is registered."""
    if (ctrl := active_controller()) is None:
        return ScopeMode.OPM
    mode = getattr(ctrl, "mode", ScopeMode.OPM)
    return mode if isinstance(mode, ScopeMode) else ScopeMode.OPM


def force_opm_if_epi() -> None:
    """Return to OPM if the app is in EPI; a no-op otherwise. Never raises."""
    if current_mode() is not ScopeMode.EPI:
        return
    force = getattr(active_controller(), "force_opm", None)
    if callable(force):
        force()


def epi_engine_available() -> bool:
    """Whether an EPI MDA engine has been hooked up (Step 4 does so)."""
    return getattr(active_controller(), "epi_engine_factory", None) is not None


def _call_ref(ref: weakref.WeakMethod[Callable[[], None]]) -> None:
    if (fn := ref()) is not None:
        fn()


class ModeController(QObject):
    """Owns the OPM/EPI mode and performs the safe transitions.

    Parameters
    ----------
    mmc : CMMCorePlus
        The main-process core.
    register_opm_engine : Callable[[], None] | None
        Reinstalls the OPM MDA engine on exit (the main window passes its
        unchanged ``_register_mda_engine``).
    parent : QObject | None
        Qt parent (the main window).

    Attributes
    ----------
    epi_engine_factory : Callable[[CMMCorePlus], MDAEngine] | None
        Step 4 sets this to build the EPI MDA engine. While ``None`` no engine
        swap happens and every MDA started in EPI is refused.
    epi_service_provider : Callable[[], object | None] | None
        Step 3b sets this to return the EPI camera service (spawning it on first
        use). Called on entry, and on exit to stop EPI Live.
    epi_service_peek : Callable[[], object | None] | None
        Returns the EPI camera service if it already exists, without creating or
        spawning it. Used when leaving EPI (falls back to ``epi_service_provider``).
    extra_preconditions : list[Callable[[], str | None]]
        Extra EPI-entry checks; each returns a refusal reason or ``None``
        (Step 3b adds "Orca settings missing").
    """

    modeChanged = Signal(object)

    def __init__(
        self,
        mmc: CMMCorePlus,
        *,
        register_opm_engine: Callable[[], None] | None = None,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._mmc = mmc
        # Held weakly when it is a bound method: the main window owns this
        # controller, and a strong reference back would form a cycle that the
        # garbage collector tears down in an arbitrary order.
        self._register_opm_engine: Callable[[], None] | None
        if register_opm_engine is not None and hasattr(register_opm_engine, "__self__"):
            ref = weakref.WeakMethod(register_opm_engine)
            self._register_opm_engine = lambda: _call_ref(ref)
        else:
            self._register_opm_engine = register_opm_engine
        self._mode = ScopeMode.OPM
        self._snapshot: TigerSnapshot | None = None
        self._busy = False
        self.epi_engine_factory: Callable[[CMMCorePlus], MDAEngine] | None = None
        self.epi_service_provider: Callable[[], object | None] | None = None
        self.epi_service_peek: Callable[[], object | None] | None = None
        self.extra_preconditions: list[Callable[[], str | None]] = []
        set_active_controller(self)

    @property
    def mode(self) -> ScopeMode:
        """The current mode (starts as OPM, never persisted)."""
        return self._mode

    # ------------------------------------------------------------------ switch

    def request_mode(self, mode: ScopeMode) -> str | None:
        """Switch to *mode*. Return ``None`` on success, else the refusal reason."""
        if mode is self._mode:
            return None
        if self._busy:
            return "A mode change is already in progress."
        self._busy = True
        try:
            reason = self._enter_epi() if mode is ScopeMode.EPI else self._leave_epi()
        finally:
            self._busy = False
        if reason is None:
            self.modeChanged.emit(self._mode)
        return reason

    def _epi_service(self) -> object | None:
        """The EPI service to check on exit; ``None`` if absent or never spawned."""
        getter = self.epi_service_peek or self.epi_service_provider
        if getter is None:
            return None
        try:
            svc = getter()
        except Exception:
            logger.exception("EPI camera service provider failed")
            return None
        # No pool, nothing to stop: a failed spawn must not block leaving EPI.
        state = getattr(getattr(svc, "state", None), "name", None)
        return None if state == "INACTIVE" else svc

    def _enter_epi(self) -> str | None:
        from pymmcore_gui.epi import _tiger_guard

        reason = _tiger_guard.check_preconditions(self._mmc, entering=True)
        if reason is not None:
            return reason
        for check in self.extra_preconditions:
            try:
                reason = check()
            except Exception as exc:
                logger.exception("EPI precondition check failed")
                reason = f"EPI precondition check failed: {exc}"
            if reason is not None:
                return reason
        try:
            self._snapshot = _tiger_guard.enter_epi(
                self._mmc,
                engine_factory=self.epi_engine_factory,
                service_provider=self.epi_service_provider,
            )
        except _tiger_guard.SwitchRefused as exc:
            return str(exc)
        self._mode = ScopeMode.EPI
        return None

    def _leave_epi(self) -> str | None:
        from pymmcore_gui.epi import _tiger_guard

        reason = _tiger_guard.check_preconditions(
            self._mmc, entering=False, epi_service=self._epi_service()
        )
        if reason is not None:
            return reason
        _tiger_guard.exit_epi(
            self._mmc,
            self._snapshot or _tiger_guard.TigerSnapshot(),
            register_opm_engine=self._register_opm_engine,
        )
        self._snapshot = None
        self._mode = ScopeMode.OPM
        return None

    def force_opm(self) -> None:
        """Return to OPM without preconditions (a config load is about to or did run).

        Tolerates LED devices that a reload already unloaded, restores nothing
        from the pre-EPI snapshot (it belongs to the old config), and never
        raises. Does nothing while an MDA is running: the engine cannot be
        swapped then, and the caller refuses the load anyway.
        """
        if self._mode is ScopeMode.OPM:
            return
        try:
            if self._mmc.mda.is_running():
                logger.warning("Cannot leave EPI mode while an MDA is running")
                return
        except Exception:
            logger.exception("Could not check for a running MDA")
        try:
            from pymmcore_gui.epi import _tiger_guard

            _tiger_guard.exit_epi(
                self._mmc,
                self._snapshot or _tiger_guard.TigerSnapshot(),
                register_opm_engine=self._register_opm_engine,
                restore_state=False,
            )
        except Exception:
            logger.exception("Error while forcing OPM mode")
        self._snapshot = None
        self._mode = ScopeMode.OPM
        logger.info("Returned to OPM mode")
        self.modeChanged.emit(self._mode)
