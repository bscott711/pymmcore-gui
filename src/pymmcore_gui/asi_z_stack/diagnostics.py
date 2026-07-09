# src/pymmcore_gui/asi_z_stack/diagnostics.py
"""Manual bench diagnostics for the ASI PLogic/galvo trigger chain.

Not used by :class:`~pymmcore_gui.asi_z_stack.engine.ASISPIMEngine` -- these
are one-off tools for hunting down hardware wiring/addressing issues on the
bench, meant to be run from an interactive console (e.g. pymmcore-gui's
embedded console, or ``python -i``) against an already-configured
``CMMCorePlus`` instance. They intentionally live outside the acquisition
path: an MDA must stay fully hardware-timed once triggered, with no serial
chatter competing with the SPIM state machine's own timing.
"""

from __future__ import annotations

import time

from pymmcore_gui._multi_camera_handler import physical_camera_labels

from .asi_controller import _HW, _send_tiger_command, mmc, set_camera_trigger_mode

# Backplane TTL0-TTL7, per ASI's PLogic docs ("addresses 41-48 are for the
# backplane signals, address 41 for TTL0 and so forth"). Address 41 (TTL0)
# has already been ruled out on the bench: with an oscilloscope on BNC1, no
# pulse appeared there while the galvo was actively scanning.
BACKPLANE_TTL_ADDRS = list(range(41, 49))


def point_bnc1_at(addr: int) -> None:
    """Route PLogic's BNC1 output directly from ``addr``, bypassing its cell.

    Lets a scope probe already on BNC1 (e.g. a "Camera Trigger" channel)
    show the raw state of any PLogic address, for hunting down which line
    actually carries the galvo's trigger signal.
    """
    plogic_addr = _HW.plogic_label.split(":")[-1]
    _send_tiger_command("M E=33", _HW.tiger_comm_hub_label)
    _send_tiger_command(f"{plogic_addr}CCA Z={addr}", _HW.tiger_comm_hub_label)
    _send_tiger_command(f"{plogic_addr}SS Z", _HW.tiger_comm_hub_label)


def restore_bnc1_to_camera_cell() -> None:
    """Undo :func:`point_bnc1_at`, restoring BNC1 to the camera NRT cell."""
    point_bnc1_at(_HW.plogic_camera_cell)


def sweep_backplane_ttl_addrs(candidates: list[int] = BACKPLANE_TTL_ADDRS) -> None:
    """Interactively step BNC1 through each backplane TTL address.

    For each candidate, points BNC1 at it and waits for you to trigger a
    scan and check the scope before moving to the next one. Always restores
    BNC1 to the camera cell when done, including on Ctrl+C.
    """
    try:
        for addr in candidates:
            point_bnc1_at(addr)
            ttl_n = addr - 41
            input(
                f"BNC1 now mirrors address {addr} (backplane TTL{ttl_n}). "
                "Trigger a scan and watch the scope, then press Enter for "
                "the next address..."
            )
    finally:
        restore_bnc1_to_camera_cell()
        print(f"BNC1 restored to the camera cell (address {_HW.plogic_camera_cell}).")


def poll_plogic_during_live_scan(
    duration_s: float = 4.0, poll_interval_s: float = 0.02
) -> None:
    """Trigger the galvo and poll PLogic's RA X?/Y?/Z? *while it is moving*.

    Every diagnostic read taken so far in this debugging session (including
    the ones ``ASISPIMEngine.setup_sequence`` logs) happens before ``PM E=1``
    and before ``SPIMState=Running`` -- i.e. before the galvo has ever moved.
    That only ever shows PLogic's idle state, which is identical every time
    and proves nothing about whether the galvo's trigger pulse ever reaches
    PLogic during an actual scan. This function is the first read taken
    *during* motion.

    Assumes the galvo and PLogic are already configured exactly as a normal
    MDA leaves them (run a real MDA once first, or at least
    ``configure_plogic_for_dual_nrt_pulses`` / the galvo SPIM properties --
    see ``ASISPIMEngine.setup_sequence`` or ``ASIStationaryTriggerEngine.
    setup_sequence``, whichever is currently registered). Triggers
    ``SPIMState=Running``,
    then polls all three RA queries every ``poll_interval_s`` for
    ``duration_s`` seconds, printing only when a value changes from the
    previous poll (with a timestamp), so a silent run is instantly obvious.

    How to read the result:

    - If nothing ever prints: PLogic observed *no* change on any of its
      front-panel, backplane, or cell-output bitmasks during the scan. The
      galvo's TTL OUT0 pulse (armed by ``TTL X=0 Y=20``) is not reaching
      PLogic in any form the ``RA`` queries can see -- point to a wiring
      problem (does a cable actually run from the galvo card to PLogic?) or
      a firmware behavior that doesn't match the documented ``Y=20``
      description, not a PLogic addressing/cell-configuration problem.
    - If ``RA X?`` or ``RA Y?`` changes but ``RA Z?`` never does: the pulse
      *is* reaching PLogic (front panel or backplane), but the dual-NRT
      cells never fire -- the problem is specifically in how the cells are
      wired to their trigger input (``CCB X=<addr>``), i.e. probably the
      wrong address, not a missing signal.
    - If ``RA Z?`` changes: at least one cell's output changed state during
      the scan -- check which bit to see whether it was the camera cell
      (``plogic_camera_cell``, default 11) or laser cell
      (``plogic_laser_on_cell``, default 10).
    """
    galvo_label = _HW.galvo_a_label
    plogic_addr = _HW.plogic_label.split(":")[-1]

    mmc.setProperty(galvo_label, "SPIMState", "Running")
    print(f"Galvo triggered -- polling PLogic RA X?/Y?/Z? for {duration_s:.1f}s...")

    last: dict[str, str | None] = {"X": None, "Y": None, "Z": None}
    t0 = time.time()
    changed = False
    while time.time() - t0 < duration_s:
        for axis in ("X", "Y", "Z"):
            resp = _send_tiger_command(
                f"{plogic_addr}RA {axis}?", _HW.tiger_comm_hub_label
            )
            if resp != last[axis]:
                elapsed = time.time() - t0
                print(
                    f"  [{elapsed:6.3f}s] RA {axis}? changed: "
                    f"{last[axis]!r} -> {resp!r}"
                )
                last[axis] = resp
                changed = True
        time.sleep(poll_interval_s)

    if not changed:
        print(
            "No change observed on RA X?/Y?/Z? for the whole scan -- PLogic "
            "never saw the galvo do anything."
        )
    mmc.setProperty(galvo_label, "SPIMState", "Idle")


def poll_camera_and_plogic_during_live_scan(
    duration_s: float = 4.0, poll_interval_s: float = 0.02, num_images: int = 402
) -> None:
    """Arm the camera for external triggering and poll it alongside PLogic.

    :func:`poll_plogic_during_live_scan` already showed PLogic's camera cell
    (BNC1, address 33) genuinely pulsing during a scan -- the galvo/PLogic
    chain this session has been debugging all day is producing real trigger
    edges. This answers the next question: does the camera ever see any of
    them? Arms the camera exactly as ``ASISPIMEngine.exec_event`` does
    (``mmc.startSequenceAcquisition``), triggers the galvo, and polls both
    the buffered-image count and PLogic's RA readbacks together.

    If PLogic keeps pulsing (``RA X?`` bit 0 flipping, same as before) but
    the buffered-image count never leaves 0, the problem is downstream of
    everything debugged so far: the physical cable from PLogic's BNC1 to the
    camera's trigger input, or the camera's own trigger-mode/polarity
    configuration -- not the galvo/PLogic setup in this codebase.

    Explicitly re-applies each physical camera's trigger mode
    (:func:`set_camera_trigger_mode`, same as
    ``ASISPIMEngine.setup_sequence``) before arming, rather than assuming
    whatever was left over from a previous run/Live view is still correct --
    an earlier version of this function skipped that and produced a
    misleading result (buffer filled rapidly with zero real BNC1 pulses,
    consistent with the camera having silently fallen back to a free-running
    trigger mode between test runs).

    Restores each camera's original ``TriggerMode`` when done (including on
    Ctrl+C/an exception), the same way :meth:`ASISPIMEngine.teardown_sequence`
    does for a real MDA -- otherwise a camera left in "Level Trigger" hangs
    the next Live/Snap waiting for a trigger that never comes.
    """
    galvo_label = _HW.galvo_a_label
    plogic_addr = _HW.plogic_label.split(":")[-1]
    active_cam = mmc.getCameraDevice()

    original_trigger_modes: dict[str, str] = {}
    for cam_label in physical_camera_labels(mmc):
        if mmc.hasProperty(cam_label, "TriggerMode"):
            original_trigger_modes[cam_label] = mmc.getProperty(
                cam_label, "TriggerMode"
            )
        set_camera_trigger_mode(cam_label)
        mode = mmc.getProperty(cam_label, "TriggerMode")
        print(f"  [{cam_label}] TriggerMode = {mode!r}")

    try:
        mmc.startSequenceAcquisition(active_cam, num_images, 0, True)
        print(f"Camera armed (sequence running: {mmc.isSequenceRunning()}).")
        mmc.setProperty(galvo_label, "SPIMState", "Running")
        print(f"Galvo triggered -- polling for {duration_s:.1f}s...")

        last: dict[str, str | None] = {"X": None, "Y": None, "Z": None}
        last_count = -1
        t0 = time.time()
        while time.time() - t0 < duration_s:
            remaining = mmc.getRemainingImageCount()
            if remaining != last_count:
                elapsed = time.time() - t0
                print(
                    f"  [{elapsed:6.3f}s] buffered images: {last_count} -> {remaining}"
                )
                last_count = remaining
            for axis in ("X", "Y", "Z"):
                resp = _send_tiger_command(
                    f"{plogic_addr}RA {axis}?", _HW.tiger_comm_hub_label
                )
                if resp != last[axis]:
                    elapsed = time.time() - t0
                    print(
                        f"  [{elapsed:6.3f}s] RA {axis}? changed: "
                        f"{last[axis]!r} -> {resp!r}"
                    )
                    last[axis] = resp
            time.sleep(poll_interval_s)

        print(f"Final buffered image count: {mmc.getRemainingImageCount()}.")
    finally:
        mmc.setProperty(galvo_label, "SPIMState", "Idle")
        mmc.stopSequenceAcquisition(active_cam)
        for cam_label, original_mode in original_trigger_modes.items():
            mmc.setProperty(cam_label, "TriggerMode", original_mode)
        print("Camera TriggerMode restored to pre-scan value(s).")


if __name__ == "__main__":
    sweep_backplane_ttl_addrs()
