"""Tests for ArgusTunnelManager's SSH command construction, including the
optional reverse admin forward used to deploy code changes from Argus."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from pymmcore_gui._argus_stream._tunnel import ArgusTunnelManager


def test_spawn_omits_reverse_forward_by_default() -> None:
    mgr = ArgusTunnelManager("Argus", 5555, 5555)
    with patch("subprocess.Popen") as popen:
        popen.return_value = MagicMock()
        mgr._spawn()
    (cmd,), _ = popen.call_args
    assert "-R" not in cmd
    assert cmd[-1] == "Argus"
    assert "-L" in cmd
    assert cmd[cmd.index("-L") + 1] == "5555:127.0.0.1:5555"


def test_spawn_adds_reverse_forward_when_configured() -> None:
    mgr = ArgusTunnelManager("Argus", 5555, 5555, reverse_port=2222)
    with patch("subprocess.Popen") as popen:
        popen.return_value = MagicMock()
        mgr._spawn()
    (cmd,), _ = popen.call_args
    assert cmd[cmd.index("-R") + 1] == "2222:localhost:22"
    # The host alias must still be the final argument (after every -o/-L/-R
    # pair), or ssh parses it as an option value instead of the target.
    assert cmd[-1] == "Argus"


def test_reverse_port_zero_behaves_exactly_like_the_old_signature() -> None:
    with_zero = ArgusTunnelManager("Argus", 5555, 5555, reverse_port=0)
    without = ArgusTunnelManager("Argus", 5555, 5555)
    with patch("subprocess.Popen") as popen:
        popen.return_value = MagicMock()
        with_zero._spawn()
        without._spawn()
    (cmd_a,), _ = popen.call_args_list[0]
    (cmd_b,), _ = popen.call_args_list[1]
    assert cmd_a == cmd_b
