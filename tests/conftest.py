"""Shared fixtures.

Everything runs against the in-process simulator, so the suite needs no robot,
no sockets and no free ports.
"""

import pytest
from mecademic_fieldbus import FieldbusRobot, get_io_map
from mecademic_fieldbus.io_map import IoMap
from mock_robot import LoopbackTransport, RobotSimulator


@pytest.fixture
def io_map() -> IoMap:
    """Return the shipped I/O map."""
    return get_io_map()


@pytest.fixture
def simulator(io_map: IoMap) -> RobotSimulator:
    """Return a simulator whose sequences complete instantly."""
    return RobotSimulator(io_map, activation_time_s=0.0, homing_time_s=0.0)


@pytest.fixture
def robot(io_map: IoMap, simulator: RobotSimulator) -> FieldbusRobot:
    """Return a facade connected to the simulator with no network in between."""
    robot = FieldbusRobot(LoopbackTransport(simulator), io_map, default_timeout_s=2.0)
    robot.Connect("in-process")
    return robot
