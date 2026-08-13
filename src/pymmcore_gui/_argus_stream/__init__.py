"""Real-time volume streaming to Argus for deskew/deconvolution.

See the plan this implements for full context: local disk saving (S: NVMe)
is unaffected -- this package connects independently to ``core.mda.events``
and streams a copy of each completed volume to Argus's real-time receiver
over a self-managed SSH tunnel, so local acquisition/saving can never be
delayed by network conditions.
"""

from ._session import ArgusStreamSession, StreamState
from ._tunnel import ArgusTunnelManager

__all__ = ["ArgusStreamSession", "ArgusTunnelManager", "StreamState"]
