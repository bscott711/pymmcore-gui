"""Scope mode (OPM vs EPI) as seen by hardware-free helpers.

This module is deliberately tiny and Qt-free. The mode controller (added with
the mode toggle) registers itself with :func:`set_active_controller`; until then
:func:`current_mode` is always ``OPM``, so tests, the demo setup and plugins see
exactly today's behaviour.
"""

from __future__ import annotations

import enum


class ScopeMode(enum.Enum):
    """Which imaging path the app is in."""

    OPM = "OPM"
    EPI = "EPI"


_active_controller: object | None = None


def set_active_controller(ctrl: object | None) -> None:
    """Register (or clear, with ``None``) the object that owns the current mode.

    The controller must expose a ``mode`` attribute holding a `ScopeMode`.
    """
    global _active_controller
    _active_controller = ctrl


def current_mode() -> ScopeMode:
    """Return the current mode; ``OPM`` when no controller is registered."""
    if _active_controller is None:
        return ScopeMode.OPM
    mode = getattr(_active_controller, "mode", ScopeMode.OPM)
    return mode if isinstance(mode, ScopeMode) else ScopeMode.OPM
