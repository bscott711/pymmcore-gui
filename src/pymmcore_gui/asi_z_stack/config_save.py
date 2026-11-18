"""Save a system configuration while the camera worker service is active.

While :class:`~pymmcore_gui.asi_z_stack.camera_worker_service.CameraWorkerService`
is active the main-process core no longer holds the camera devices (nor the
``Multi Camera`` composite) and the config groups that reference them were
deleted from it, so the plain ``saveSystemConfiguration`` writes a ``.cfg``
without any of those. Saving over the loaded file would leave the rig with no
cameras on the next start.

:func:`save_system_configuration` runs the stock writer and then splices the
missing pieces back into the saved text: the released ``Device`` lines (and
their pre-init ``Property``/``Parent`` and post-init
``Delay``/``FocusDirection``/``Label`` lines, copied from the file that was
loaded), the worker-owned config groups (with any in-memory edits) and the
``Camera`` role.
"""

from __future__ import annotations

import contextlib
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

from pymmcore_gui.asi_z_stack.camera_worker_service import CameraWorkerService

if TYPE_CHECKING:
    from collections.abc import Mapping

    from pymmcore_plus import CMMCorePlus

    from pymmcore_gui.asi_z_stack.camera_worker_service import WorkerConfigGroup

_INIT_LINE = "Property,Core,Initialize,1"
_CAMERA_ROLE_PREFIX = "Property,Core,Camera,"
_POST_INIT_KINDS = frozenset({"Delay", "FocusDirection", "Label"})


@dataclass
class _ReleasedLines:
    """cfg lines of the released devices, grouped by where they are spliced."""

    devices: list[str] = field(default_factory=list)
    pre_init: list[str] = field(default_factory=list)
    post_init: list[str] = field(default_factory=list)


def save_system_configuration(mmc: CMMCorePlus, path: str | os.PathLike[str]) -> None:
    """Save the system configuration to *path*, camera-worker aware.

    With no active camera worker service this is exactly
    ``mmc.saveSystemConfiguration(path)``. With one active, the stock save is
    followed by splicing the released camera devices, the worker-owned config
    groups and the ``Camera`` role back into the file (see the module
    docstring). The file is replaced atomically.

    Parameters
    ----------
    mmc : CMMCorePlus
        The core to save.
    path : str | os.PathLike[str]
        Destination ``.cfg`` file (may be the file that was loaded).

    Raises
    ------
    RuntimeError
        If the service is active but the file that was loaded cannot be read
        (the released devices could not be restored), or the saved text has no
        ``Property,Core,Initialize,1`` line. Nothing is written in that case.
    """
    svc = CameraWorkerService.get_active()
    geometry = svc.geometry if svc is not None else None
    if svc is None or geometry is None:
        mmc.saveSystemConfiguration(os.fspath(path))
        return

    released = set(svc.camera_labels)
    role = geometry.core_camera_role
    if geometry.n_cameras > 1 and role:
        released.add(role)

    # Read the source BEFORE saving: the target may be the same file.
    source = _read_text(_loaded_config_path(mmc))
    lines = _collect_released_lines(source, released)

    dest = Path(path)
    fd, tmp_name = tempfile.mkstemp(dir=dest.parent, prefix=dest.name, suffix=".tmp")
    os.close(fd)
    try:
        mmc.saveSystemConfiguration(tmp_name)
        saved = _read_text(tmp_name)
        merged = splice_worker_state(saved, lines, svc.worker_config_groups, role)
        with open(tmp_name, "w", encoding="utf-8", errors="surrogateescape") as f:
            f.write(merged)
        os.replace(tmp_name, dest)
    except BaseException:
        with contextlib.suppress(OSError):
            os.unlink(tmp_name)
        raise


def _loaded_config_path(mmc: CMMCorePlus) -> str:
    path = mmc.systemConfigurationFile()
    if not path:
        raise RuntimeError(
            "Cannot save the configuration: the camera devices are owned by "
            "the camera workers and no configuration file was loaded to "
            "restore them from."
        )
    return path


def _read_text(path: str | os.PathLike[str]) -> str:
    """Read *path* verbatim (newlines untouched), tolerating odd bytes."""
    try:
        with open(path, encoding="utf-8", errors="surrogateescape", newline="") as f:
            return f.read()
    except OSError as exc:
        raise RuntimeError(
            f"Cannot save the configuration: unable to read {os.fspath(path)!r} "
            f"to restore the camera devices ({exc})."
        ) from exc


def _label(line: str) -> tuple[str, str | None]:
    """Return ``(kind, label)`` of a cfg line (``label`` is None if absent)."""
    parts = line.strip().split(",", 2)
    return parts[0], (parts[1] if len(parts) > 1 else None)


def _collect_released_lines(source: str, released: set[str]) -> _ReleasedLines:
    """Collect, in source order, the cfg lines belonging to *released* labels."""
    out = _ReleasedLines()
    initialized = False
    for line in source.splitlines():
        if line.strip() == _INIT_LINE:
            initialized = True
            continue
        kind, label = _label(line)
        if label not in released:
            continue
        if kind == "Device":
            out.devices.append(line)
        elif not initialized and kind in ("Property", "Parent"):
            out.pre_init.append(line)
        elif initialized and kind in _POST_INIT_KINDS:
            out.post_init.append(line)
    return out


def splice_worker_state(
    saved: str,
    released: _ReleasedLines,
    groups: Mapping[str, WorkerConfigGroup],
    camera_role: str,
) -> str:
    """Splice the released devices, worker groups and Camera role into *saved*.

    Parameters
    ----------
    saved : str
        Text written by the stock ``saveSystemConfiguration``.
    released : _ReleasedLines
        Lines of the released devices, copied from the loaded file.
    groups : Mapping[str, WorkerConfigGroup]
        Worker-owned config groups (replace any same-named group in *saved*).
    camera_role : str
        Value for ``Property,Core,Camera`` if *saved* has none (may be empty).

    Returns
    -------
    str
        The merged text, using the newline convention of *saved*.

    Raises
    ------
    RuntimeError
        If *saved* has no ``Property,Core,Initialize,1`` line.
    """
    newline = "\r\n" if "\r\n" in saved else "\n"
    lines = saved.splitlines()
    try:
        init = next(i for i, ln in enumerate(lines) if ln.strip() == _INIT_LINE)
    except StopIteration:
        raise RuntimeError(
            f"Cannot save the configuration: the saved text has no {_INIT_LINE!r} "
            "line to anchor the camera devices to."
        ) from None

    # Devices go after the last Device line; if there is none (before init),
    # fall back to just before the pre-init properties.
    last_dev = max(
        (i for i, ln in enumerate(lines[:init]) if _label(ln)[0] == "Device"),
        default=init - 1,
    )
    merged = [
        *lines[: last_dev + 1],
        *released.devices,
        *lines[last_dev + 1 : init],
        *released.pre_init,
        lines[init],
        *released.post_init,
        *lines[init + 1 :],
    ]

    names = set(groups)
    merged = [
        ln
        for ln in merged
        if not (_label(ln)[0] == "ConfigGroup" and _label(ln)[1] in names)
    ]

    block = [
        f"ConfigGroup,{group},{preset},{dev},{prop},{val}"
        for group, wg in groups.items()
        for preset, settings in wg.presets.items()
        for dev, prop, val in settings
    ]
    if block:
        merged.append("# Camera worker config groups")
        merged.extend(block)
    if camera_role and not any(ln.startswith(_CAMERA_ROLE_PREFIX) for ln in merged):
        merged.append("# Camera role")
        merged.append(f"{_CAMERA_ROLE_PREFIX}{camera_role}")
    return newline.join(merged) + newline
