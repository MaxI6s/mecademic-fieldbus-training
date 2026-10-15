"""Build the connection the training tools drive.

Both tools can run against three things: an in-process simulator, a simulated
robot listening on sockets, or a real one.  Choosing between them is the same
decision in both, so it is made here once.

The one wrinkle worth centralising is the receiving port.  A scanner listens on
UDP 2222 by default, which is also the port a simulated robot on the same
machine binds to -- so pointing a tool at ``127.0.0.1`` while ``python -m
mock_robot`` is running fails with "address already in use".  That is an
unhelpful first experience for a training tool, so a loopback target quietly
gets an ephemeral port instead, and says so.
"""

from typing import Any, Optional, Tuple

from mecademic_fieldbus.io_map import IoMap
from mecademic_fieldbus.transports.base import FieldbusTransport

__all__ = [
    "build_transport",
    "describe_target",
    "is_loopback",
    "resolve_udp_port",
    "RecordingTransport",
]


def is_loopback(address: Optional[str]) -> bool:
    """Tell whether an address points back at this machine.

    Args:
        address: Address to classify, or ``None`` for the in-process simulator.

    Returns:
        ``True`` for anything in ``127.0.0.0/8`` or ``localhost``.
    """
    if address is None:
        return False
    return address == "localhost" or address.split(".")[0] == "127"


def resolve_udp_port(address: Optional[str], requested: Optional[int]) -> Tuple[Optional[int], str]:
    """Decide which UDP port the scanner should listen on.

    Args:
        address: Address being connected to.
        requested: Port asked for on the command line, or ``None``.

    Returns:
        A tuple ``(port, note)``.  The port is ``None`` to accept the library's
        default; the note explains any choice made on the caller's behalf, and
        is empty when none was.
    """
    if requested is not None:
        return requested, ""
    if is_loopback(address):
        return 0, (
            "note: {} is on this machine, so a simulated robot there already holds the "
            "standard UDP 2222. Listening on a free port instead; pass --udp-port 2222 "
            "to override.".format(address)
        )
    return None, ""


def describe_target(address: Optional[str]) -> str:
    """Name the target, for a startup line.

    Args:
        address: Address being connected to, or ``None``.

    Returns:
        A short description.
    """
    return address if address else "the in-process simulator"


def build_transport(
    io_map: IoMap,
    address: Optional[str] = None,
    udp_port: Optional[int] = None,
    activation_time_s: float = 0.5,
    homing_time_s: float = 1.0,
) -> Tuple[FieldbusTransport, Optional[Any], str]:
    """Build the transport a tool should drive.

    Args:
        io_map: Map describing the assemblies.
        address: Address of a robot, or ``None`` for the in-process simulator.
        udp_port: Port to listen on, or ``None`` to decide automatically.
        activation_time_s: Simulated activation duration, when simulating.
        homing_time_s: Simulated homing duration, when simulating.

    Returns:
        A tuple ``(transport, simulator, note)``.  The simulator is ``None``
        unless the transport is the in-process one, and the note is a line to
        print before connecting, empty when there is nothing to say.

    Raises:
        FieldbusConnectionError: If the EtherNet/IP stack is unavailable.
    """
    if address is None:
        from mock_robot import LoopbackTransport, RobotSimulator

        simulator = RobotSimulator(
            io_map, activation_time_s=activation_time_s, homing_time_s=homing_time_s
        )
        return LoopbackTransport(simulator), simulator, ""

    from mecademic_fieldbus.transports.ethernetip import EtherNetIpTransport

    port, note = resolve_udp_port(address, udp_port)
    overrides = {} if port is None else {"originator_udp_port": port}
    return EtherNetIpTransport.from_io_map(io_map, **overrides), None, note


class RecordingTransport(FieldbusTransport):
    """Remember the last output image written through a transport.

    The dashboard shows what is being produced to the robot.  While a scenario
    runs, the writer is the walkthrough rather than the facade, and each keeps
    its own latched image -- so reading the image back from either one shows
    the wrong thing half the time.  Recording it at the transport, where every
    writer meets, leaves one source of truth.

    Args:
        inner: Transport to delegate to.
        initial_output: Image to report before anything has been written.
    """

    def __init__(self, inner: FieldbusTransport, initial_output: bytes) -> None:
        self._inner = inner
        self._last_output = initial_output

    @property
    def inner(self) -> FieldbusTransport:
        """The transport being wrapped."""
        return self._inner

    @property
    def last_output(self) -> bytes:
        """The most recent image written, whoever wrote it."""
        return self._last_output

    def connect(self, address: str, **kwargs: Any) -> None:
        """Connect the wrapped transport.

        Args:
            address: Address to connect to.
            **kwargs: Passed through.
        """
        self._inner.connect(address, **kwargs)

    def disconnect(self) -> None:
        """Disconnect the wrapped transport."""
        self._inner.disconnect()

    @property
    def is_connected(self) -> bool:
        """Whether the wrapped transport is connected."""
        return self._inner.is_connected

    def read_input_assembly(self) -> bytes:
        """Return the latest input image.

        Returns:
            The raw image, straight from the wrapped transport.
        """
        return self._inner.read_input_assembly()

    def write_output_assembly(self, data: bytes) -> None:
        """Record an output image and pass it on.

        Args:
            data: Raw output assembly image.
        """
        self._inner.write_output_assembly(data)
        self._last_output = bytes(data)
