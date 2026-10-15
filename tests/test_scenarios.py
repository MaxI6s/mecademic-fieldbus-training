"""Tests of the scenario machinery the dashboard displays.

The scenarios are written once, in :mod:`training.command_walkthrough`, and
rendered twice: as narrated text in the terminal and as a step sequence in the
browser.  These tests pin the declarative half, which is what the page reads.
"""

import json
import threading
import time
import urllib.error
import urllib.request

import pytest
from mecademic_fieldbus import FieldbusRobot, get_io_map
from mecademic_fieldbus.io_map import INPUT_ASSEMBLY, OUTPUT_ASSEMBLY, IoMap

from training import tag_dashboard
from training.assembly_view import AssemblyView
from training.backend import RecordingTransport
from training.command_walkthrough import (
    SCENARIO_GROUPS,
    Walkthrough,
    available_scenarios,
    describe_scenarios,
    prepare,
)


# ----------------------------------------------------------------------
# The declarative description
# ----------------------------------------------------------------------
def test_every_step_names_tags_that_exist(io_map: IoMap) -> None:
    """A step may only name tags the assemblies actually carry.

    Without this a typo renders as an empty value in the page rather than as
    an error, which in a training tool teaches a tag name that does not exist.
    """
    inputs = {field.name for field in io_map.describe_fields(INPUT_ASSEMBLY)}
    outputs = {field.name for field in io_map.describe_fields(OUTPUT_ASSEMBLY)}
    for scenario in describe_scenarios(io_map):
        for step in scenario["steps"]:
            for name in step["sets"]:
                assert name in outputs, "{}/{}: {}".format(scenario["name"], step["title"], name)
            for name in step["waits"]:
                assert name in inputs, "{}/{}: {}".format(scenario["name"], step["title"], name)


def test_every_step_that_blocks_names_what_it_waits_on(io_map: IoMap) -> None:
    """A step that waits must say which tag carries the answer.

    That pairing -- this bit out, that bit back -- is the whole lesson, so a
    blocking step with nothing to show is a gap in the material.
    """
    for scenario in describe_scenarios(io_map):
        for step in scenario["steps"]:
            if step["blocking"]:
                assert step["waits"], "{}/{}".format(scenario["name"], step["title"])


def test_no_step_writes_a_tag_it_did_not_declare(io_map: IoMap) -> None:
    """Every output field a step actually changes is one the step declares.

    The declared tags are what the page draws in its SET card and what the
    terminal narrates, so an undeclared write is a change an integrator watches
    happen with no explanation attached.  Rather than trust the declaration,
    this runs each step and compares it against what moved.
    """
    from mock_robot import LoopbackTransport, RobotSimulator

    output_view = AssemblyView(io_map, OUTPUT_ASSEMBLY)
    for scenario in available_scenarios(io_map).values():
        simulator = RobotSimulator(io_map, activation_time_s=0.0, homing_time_s=0.0)
        transport = LoopbackTransport(simulator)
        transport.connect("in-process")
        walkthrough = Walkthrough(transport, io_map, color=False, settle_timeout_s=5.0)
        walkthrough.simulator = simulator
        prepare(walkthrough, scenario.requires, timeout_s=5.0)
        for step in scenario.steps:
            before = walkthrough.output_image
            step.action(walkthrough)
            moved = {
                change.field.name
                for change in output_view.changed_fields(before, walkthrough.output_image)
            }
            undeclared = moved - set(step.sets)
            assert not undeclared, "{}/{} writes {} without declaring it".format(
                scenario.name, step.title, sorted(undeclared)
            )


def test_every_scenario_belongs_to_a_declared_group(io_map: IoMap) -> None:
    """A scenario in no group would have no tab to live in."""
    groups = {name for name, _, _ in SCENARIO_GROUPS}
    for scenario in available_scenarios(io_map).values():
        assert scenario.group in groups, scenario.name


def test_every_group_has_at_least_one_scenario(io_map: IoMap) -> None:
    """An empty group would render as a heading with nothing under it."""
    used = {scenario.group for scenario in available_scenarios(io_map).values()}
    for name, _, _ in SCENARIO_GROUPS:
        assert name in used, name


def test_a_prerequisite_is_reached_before_the_steps(io_map: IoMap) -> None:
    """A sequence that assumes a state gets there first, and says so.

    Each sequence shows one command; reaching the state it assumes is
    preparation, and a sequence left to start it in step one would be teaching
    activation over and over instead.
    """
    from mock_robot import LoopbackTransport, RobotSimulator

    for requires, check in (
        ("activated", lambda status: status.activated),
        ("homed", lambda status: status.homed),
    ):
        simulator = RobotSimulator(io_map, activation_time_s=0.0, homing_time_s=0.0)
        transport = LoopbackTransport(simulator)
        transport.connect("in-process")
        walkthrough = Walkthrough(transport, io_map, color=False)
        assert not check(io_map.decode_status(walkthrough.input_image()))
        done = prepare(walkthrough, requires, timeout_s=5.0)
        assert check(io_map.decode_status(walkthrough.input_image())), requires
        assert requires.rstrip("d") in done, done


def test_the_description_matches_the_scenarios(io_map: IoMap) -> None:
    """The description covers every scenario, with the same steps and grouping."""
    scenarios = available_scenarios(io_map)
    described = describe_scenarios(io_map)
    assert [entry["name"] for entry in described] == list(scenarios)
    for entry in described:
        scenario = scenarios[entry["name"]]
        assert len(entry["steps"]) == len(scenario.steps)
        assert entry["group"] == scenario.group
        assert entry["label"] == scenario.label
        assert entry["requires"] == scenario.requires


# ----------------------------------------------------------------------
# The recording transport
# ----------------------------------------------------------------------
def test_the_recording_transport_reports_the_last_image_written(io_map: IoMap, simulator) -> None:
    """Whoever writes, the recorded image is what went to the robot.

    The dashboard shows the produced image; while a scenario runs the writer is
    the walkthrough rather than the facade, so reading it back from either one
    alone would show the wrong thing half the time.
    """
    from mock_robot import LoopbackTransport

    recording = RecordingTransport(LoopbackTransport(simulator), io_map.empty_output_assembly())
    recording.connect("in-process")
    assert recording.last_output == io_map.empty_output_assembly()

    robot = FieldbusRobot(recording, io_map, default_timeout_s=2.0)
    robot.Connect("in-process")
    robot.ActivateRobot()
    assert io_map.decode_robot_control(recording.last_output).activate is True

    # Now a second writer, as a running scenario would be.
    walkthrough = Walkthrough(recording, io_map, color=False)
    walkthrough.set_robot_control(home=True)
    control = io_map.decode_robot_control(recording.last_output)
    assert control.home is True


# ----------------------------------------------------------------------
# The runner
# ----------------------------------------------------------------------
def test_the_runner_walks_every_step(io_map: IoMap) -> None:
    """A scenario runs to completion and reports each step as it lands.

    The simulator is given a realistic activation delay, because a robot that
    transitions in no time at all is the one case the steps must not assume.
    """
    from mock_robot import LoopbackTransport, RobotSimulator

    simulator = RobotSimulator(io_map, activation_time_s=0.1, homing_time_s=0.1)
    recording = RecordingTransport(LoopbackTransport(simulator), io_map.empty_output_assembly())
    recording.connect("in-process")
    walkthrough = Walkthrough(recording, io_map, color=False)
    walkthrough.simulator = simulator
    runner = tag_dashboard.ScenarioRunner(
        walkthrough,
        available_scenarios(io_map),
        ("activate",),
        threading.Lock(),
        step_delay_s=0.0,
        settle_timeout_s=5.0,
    )

    started, _ = runner.start("activate")
    assert started is True

    deadline = time.monotonic() + 30.0
    while runner.running and time.monotonic() < deadline:
        time.sleep(0.02)
    state = runner.state
    assert state.status == "done", state.message
    assert set(state.steps) == {"done"}


def test_an_instant_robot_does_not_hang_a_step(io_map: IoMap, simulator) -> None:
    """A robot that activates between two polls still satisfies step one.

    The fixture simulator transitions instantly, so ``RobotStatus_Busy`` is
    never observed; the step settles on the robot having reacted at all.
    """
    from mock_robot import LoopbackTransport

    recording = RecordingTransport(LoopbackTransport(simulator), io_map.empty_output_assembly())
    recording.connect("in-process")
    walkthrough = Walkthrough(recording, io_map, color=False)
    walkthrough.simulator = simulator
    runner = tag_dashboard.ScenarioRunner(
        walkthrough,
        available_scenarios(io_map),
        ("activate",),
        threading.Lock(),
        step_delay_s=0.0,
        settle_timeout_s=3.0,
    )
    runner.start("activate")
    deadline = time.monotonic() + 20.0
    while runner.running and time.monotonic() < deadline:
        time.sleep(0.02)
    assert runner.state.status == "done", runner.state.message


def test_the_runner_refuses_a_scenario_that_is_not_enabled(io_map: IoMap, simulator) -> None:
    """Only the scenarios the command line allowed can be started."""
    from mock_robot import LoopbackTransport

    recording = RecordingTransport(LoopbackTransport(simulator), io_map.empty_output_assembly())
    recording.connect("in-process")
    runner = tag_dashboard.ScenarioRunner(
        Walkthrough(recording, io_map, color=False),
        available_scenarios(io_map),
        ("activate",),
        threading.Lock(),
        step_delay_s=0.0,
    )
    started, message = runner.start("send-command")
    assert started is False
    assert "not enabled" in message


def test_allowed_scenarios_follow_the_command_tiers(io_map: IoMap) -> None:
    """Scenarios are gated exactly like the commands are."""
    parser = tag_dashboard.build_parser()
    assert tag_dashboard.allowed_scenarios(parser.parse_args([]), io_map) == ()

    safe = tag_dashboard.allowed_scenarios(parser.parse_args(["--allow-commands"]), io_map)
    assert "activate" in safe
    assert "send-command" not in safe and "velocity" not in safe

    moving = tag_dashboard.allowed_scenarios(parser.parse_args(["--allow-motion"]), io_map)
    assert "send-command" in moving

    on_robot = tag_dashboard.allowed_scenarios(
        parser.parse_args(["--allow-motion", "--address", "192.168.0.100"]), io_map
    )
    assert "clear-fault" not in on_robot, "a real robot will not fault on request"


# ----------------------------------------------------------------------
# The routes
# ----------------------------------------------------------------------
def serve(robot: FieldbusRobot, scenarios: tuple):
    """Serve a dashboard with a scenario runner, on an ephemeral port.

    Args:
        robot: Facade to drive.
        scenarios: Scenario names to enable.

    Returns:
        A tuple ``(server, port, sampler)``.
    """
    io_map = robot.io_map
    sampler = tag_dashboard.TagSampler(robot, poll_interval_s=0.02)
    runner = None
    if scenarios:
        walkthrough = Walkthrough(robot.transport, io_map, color=False)
        runner = tag_dashboard.ScenarioRunner(
            walkthrough, available_scenarios(io_map), scenarios, threading.Lock(), 0.0
        )
    handler = tag_dashboard.make_handler(
        tag_dashboard.build_layout_document(io_map, (), scenarios),
        sampler,
        str(tag_dashboard.__file__).replace("tag_dashboard.py", "tag_dashboard.html"),
        None,
        runner,
    )
    server = tag_dashboard.ThreadingServer(("127.0.0.1", 0), handler)
    sampler.start()
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, server.server_address[1], sampler


def test_the_layout_carries_the_scenarios(robot: FieldbusRobot) -> None:
    """The page gets the sequences as data, so it renders none of its own."""
    server, port, sampler = serve(robot, ("activate",))
    try:
        with urllib.request.urlopen("http://127.0.0.1:{}/layout".format(port), timeout=5) as reply:
            layout = json.loads(reply.read())
        assert [s["name"] for s in layout["scenarios"]] == list(available_scenarios(robot.io_map))
        assert layout["allowedScenarios"] == ["activate"]
    finally:
        server.shutdown()
        server.server_close()
        sampler.stop()


def test_a_read_only_dashboard_has_no_scenario_endpoint(robot: FieldbusRobot) -> None:
    """With no scenario enabled the route does not exist at all."""
    server, port, sampler = serve(robot, ())
    try:
        with urllib.request.urlopen("http://127.0.0.1:{}/layout".format(port), timeout=5) as reply:
            assert json.loads(reply.read())["allowedScenarios"] == []
        request = urllib.request.Request(
            "http://127.0.0.1:{}/scenario".format(port),
            data=json.dumps({"name": "activate"}).encode(),
            headers={"Content-Type": "application/json"},
        )
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(request, timeout=5)
        assert error.value.code == 404
    finally:
        server.shutdown()
        server.server_close()
        sampler.stop()


def test_the_stream_carries_the_scenario_progress(robot: FieldbusRobot) -> None:
    """Progress rides the same stream as the tag values, so they stay in step."""
    server, port, sampler = serve(robot, ("activate",))
    try:
        request = urllib.request.Request(
            "http://127.0.0.1:{}/scenario".format(port),
            data=json.dumps({"name": "activate"}).encode(),
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(request, timeout=5) as reply:
            assert json.loads(reply.read())["ok"] is True

        seen = []
        with urllib.request.urlopen(
            "http://127.0.0.1:{}/stream".format(port), timeout=10
        ) as stream:
            buffer = b""
            while len(seen) < 6:
                buffer += stream.read(1)
                if buffer.endswith(b"\n\n"):
                    for line in buffer.decode().splitlines():
                        if line.startswith("data: "):
                            seen.append(json.loads(line[6:])["scenario"])
                    buffer = b""
        assert all(state is not None for state in seen)
        assert {state["name"] for state in seen} == {"activate"}
        assert any(state["status"] in ("running", "waiting", "done") for state in seen)
    finally:
        server.shutdown()
        server.server_close()
        sampler.stop()


def test_every_scenario_describes_its_process(io_map: IoMap) -> None:
    """Each sequence carries the paragraph the terminal walkthrough leads with.

    The page shows the facts below instead; the terminal, where reading is the
    point, prints this.
    """
    for entry in describe_scenarios(io_map):
        assert entry["process"], entry["name"]
        assert len(entry["process"].split()) >= 20, entry["name"]
        assert "\n" not in entry["process"], entry["name"]


def test_a_finished_sequence_keeps_what_each_step_cost(io_map: IoMap) -> None:
    """Every answered step keeps its own time and read count.

    The live pair belongs to the step in progress, so without this a finished
    sequence would show no timings at all -- the number an integrator sizing a
    PLC scan is most likely to want once it is over.
    """
    from mock_robot import LoopbackTransport, RobotSimulator

    simulator = RobotSimulator(io_map, activation_time_s=0.1, homing_time_s=0.1)
    recording = RecordingTransport(LoopbackTransport(simulator), io_map.empty_output_assembly())
    recording.connect("in-process")
    walkthrough = Walkthrough(recording, io_map, color=False)
    walkthrough.simulator = simulator
    runner = tag_dashboard.ScenarioRunner(
        walkthrough,
        available_scenarios(io_map),
        ("activate",),
        threading.Lock(),
        step_delay_s=0.0,
        settle_timeout_s=5.0,
    )
    runner.start("activate")
    deadline = time.monotonic() + 30.0
    while runner.running and time.monotonic() < deadline:
        time.sleep(0.02)

    state = runner.state
    assert state.status == "done", state.message
    assert len(state.timings) == len(available_scenarios(io_map)["activate"].steps)
    blocking = [
        index
        for index, step in enumerate(available_scenarios(io_map)["activate"].steps)
        if step.settled is not None
    ]
    assert blocking, "this scenario must have a step that waits"
    for index in blocking:
        assert state.timings[index][1] >= 1, "step {} recorded no reads".format(index + 1)


#: The control *bits*.  MotionControl_MoveId is a counter rather than a request,
#: and DynamicTypeCfg0..3 are configuration words whose reset would blank the
#: very values a sequence just read, so neither is expected back at zero.
_CONTROL_BITS = (
    ("decode_robot_control", ("activate", "deactivate", "home", "reset_error")),
    ("decode_motion_control", ("setpoint", "pause", "clear_move", "resume_motion")),
)


@pytest.mark.parametrize("name", sorted(available_scenarios(get_io_map())))
def test_every_sequence_releases_the_control_bits_it_set(name: str) -> None:
    """A sequence leaves no control bit set behind it.

    Every RobotControl bit is a request the robot acts on while it is held, so
    one left set keeps acting: a held Activate reactivates a robot that a fault
    reset brought down.  Each sequence releases what it set once the robot has
    confirmed it, and the next one starts from a clean output image.
    """
    from mock_robot import LoopbackTransport, RobotSimulator

    io_map = get_io_map()
    simulator = RobotSimulator(io_map, activation_time_s=0.0, homing_time_s=0.0)
    transport = LoopbackTransport(simulator)
    transport.connect("in-process")
    walkthrough = Walkthrough(transport, io_map, color=False, settle_timeout_s=5.0)
    walkthrough.simulator = simulator

    assert walkthrough.run(available_scenarios(io_map)[name], False, 0.0) == 0
    for decoder, bits in _CONTROL_BITS:
        control = getattr(io_map, decoder)(walkthrough.output_image)
        left = [bit for bit in bits if getattr(control, bit)]
        assert not left, "{} leaves {} set".format(name, left)


def test_releasing_activate_leaves_the_robot_activated() -> None:
    """Releasing the request does not undo it.

    The sequence used to claim the opposite -- that a released Activate powers
    the robot down -- which the vendor wording does not say and the separate
    Deactivate bit contradicts.  Pinned so it cannot come back.
    """
    from mock_robot import LoopbackTransport, RobotSimulator

    io_map = get_io_map()
    simulator = RobotSimulator(io_map, activation_time_s=0.0, homing_time_s=0.0)
    transport = LoopbackTransport(simulator)
    transport.connect("in-process")
    walkthrough = Walkthrough(transport, io_map, color=False)

    walkthrough.set_robot_control(activate=True, deactivate=False)
    prepare(walkthrough, "activated", timeout_s=5.0)
    walkthrough.set_robot_control(activate=False)
    for _ in range(20):
        status = io_map.decode_status(walkthrough.input_image())
    assert status.activated, "a released Activate must not deactivate the robot"


_GLYPHS = {"level", "edge", "handshake", "held", "slot"}


def test_every_scenario_has_its_three_facts(io_map: IoMap) -> None:
    """The tab's description is three tiles, so all three must be there.

    Each one is a phrase, not a paragraph -- the page has no room for more, and
    the point of the tiles is that they read at a glance.
    """
    for entry in describe_scenarios(io_map):
        assert entry["kind"] in _GLYPHS, entry["name"]
        for key in ("mechanic", "confirms", "watch"):
            assert entry[key], "{} has no {}".format(entry["name"], key)
            assert len(entry[key]) <= 70, "{}.{} is a sentence, not a fact".format(
                entry["name"], key
            )


def test_the_confirming_tag_is_one_the_robot_publishes(io_map: IoMap) -> None:
    """ "Confirmed by" must name a real input tag, or it teaches a fiction."""
    inputs = {field.name for field in io_map.describe_fields(INPUT_ASSEMBLY)}
    for entry in describe_scenarios(io_map):
        first = entry["confirms"].split()[0].rstrip(",")
        assert first in inputs, "{} is confirmed by {!r}".format(entry["name"], first)
