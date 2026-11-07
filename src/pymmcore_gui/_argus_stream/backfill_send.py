r"""Send finished acquisitions from this PC to Argus over SSH.

A thin wrapper: the sender itself is ``opym.stream.backfill_client`` (the opym repo),
the one copy of it. This module only supplies what is specific to this app: the
``ArgusTunnelManager`` that opens the SSH tunnels, and the ``ssh_host`` and
``stream_links`` the live stream already uses (Settings).

It runs in this app's own environment, so the PC needs nothing else::

    uv run --no-sync python -m pymmcore_gui._argus_stream.backfill_send ^
        --map S:\\=/mmfs1/scratch/jacks.local/microscopy --dry-run

The flags are opym's (``--help`` lists them). The environment must hold opym, so run
``uv sync`` after pulling a change of the opym pin, before the first ``--no-sync`` run.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from opym.stream.backfill_client import SenderDefaults
from opym.stream.backfill_client import main as _opym_main

if TYPE_CHECKING:
    from collections.abc import Sequence


def sender_defaults() -> SenderDefaults:
    """The stream's ``ssh_host`` and ``stream_links``; opym's defaults if unreadable."""
    try:
        from pymmcore_gui._settings import Settings

        stream = Settings.instance().argus_stream
        return SenderDefaults(
            ssh_host=stream.ssh_host, links=max(1, stream.stream_links)
        )
    except Exception:  # settings unreadable: the stream's defaults
        return SenderDefaults()


def main(argv: Sequence[str] | None = None) -> int:
    """Run the opym sender with this app's tunnel and Settings; return the exit code."""
    from ._tunnel import ArgusTunnelManager

    return _opym_main(
        None if argv is None else list(argv),
        tunnel_factory=ArgusTunnelManager,
        defaults=sender_defaults(),
    )


if __name__ == "__main__":
    raise SystemExit(main())
