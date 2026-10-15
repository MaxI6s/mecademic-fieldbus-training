"""Tests of how a tool decides what to connect to.

The port rule is the one worth pinning: a scanner listens on UDP 2222 by
default, which is also what a simulated robot on the same machine binds, so
pointing a tool at the loopback interface used to fail with "address already in
use" and nothing else.
"""

import pytest
from mecademic_fieldbus.io_map import IoMap
from mock_robot import LoopbackTransport

from training.backend import build_transport, describe_target, is_loopback, resolve_udp_port


@pytest.mark.parametrize(
    "address, expected",
    [
        ("127.0.0.1", True),
        ("127.1.2.3", True),
        ("localhost", True),
        ("192.168.0.100", False),
        ("10.0.0.5", False),
        (None, False),
    ],
)
def test_loopback_detection(address, expected) -> None:
    """Only an address that points back at this machine counts as loopback."""
    assert is_loopback(address) is expected


def test_a_loopback_target_gets_a_free_port_and_says_so() -> None:
    """The simulated robot holds 2222, so the scanner must listen elsewhere."""
    port, note = resolve_udp_port("127.0.0.1", None)
    assert port == 0
    assert "2222" in note and "--udp-port" in note


def test_an_explicit_port_is_always_honoured() -> None:
    """Asking for a port on the command line overrides the guess, silently."""
    assert resolve_udp_port("127.0.0.1", 2222) == (2222, "")
    assert resolve_udp_port("192.168.0.100", 0) == (0, "")


def test_a_remote_target_keeps_the_library_default() -> None:
    """Against a real robot nothing is guessed: the standard port is right."""
    assert resolve_udp_port("192.168.0.100", None) == (None, "")


def test_no_address_builds_the_in_process_transport(io_map: IoMap) -> None:
    """With no address the tools run with no sockets at all."""
    transport, simulator, note = build_transport(io_map)
    assert isinstance(transport, LoopbackTransport)
    assert simulator is not None
    assert note == ""


def test_describe_target_names_the_simulator() -> None:
    """The startup line says what is being driven."""
    assert describe_target(None) == "the in-process simulator"
    assert describe_target("192.168.0.100") == "192.168.0.100"
