"""Tests of the two tools themselves.

The walkthrough is run end to end against the in-process simulator; the
dashboard is served on an ephemeral port and driven over HTTP.  Neither needs a
robot, and neither needs a fixed port.
"""

import json
import os
import threading
import urllib.error
import urllib.request

import pytest
from mecademic_fieldbus import FieldbusRobot, get_io_map
from mecademic_fieldbus.io_map import IoMap

from training import command_walkthrough, tag_dashboard

#: Arguments that make a walkthrough run unattended and quickly, while leaving
#: the activation long enough for the Busy transition to be observable.
FAST = [
    "--auto",
    "--step-delay",
    "0",
    "--no-color",
    "--settle-timeout",
    "3",
    "--activation-time",
    "0.1",
    "--homing-time",
    "0.1",
]


# ----------------------------------------------------------------------
# Walkthrough
# ----------------------------------------------------------------------
def test_listing_the_scenarios_succeeds(capsys: pytest.CaptureFixture) -> None:
    """``--list-scenarios`` prints every scenario and exits cleanly."""
    assert command_walkthrough.main(["--list-scenarios"]) == 0
    printed = capsys.readouterr().out
    for scenario in command_walkthrough.available_scenarios(get_io_map()).values():
        assert scenario.name in printed


@pytest.mark.parametrize("scenario", sorted(command_walkthrough.available_scenarios(get_io_map())))
def test_every_scenario_runs_against_the_simulator(scenario: str) -> None:
    """Each scenario completes against the in-process simulator.

    This is what makes the toolkit safe to run live: the whole sequence has
    already been executed, start to finish, with no hardware involved.  The
    list is taken from the scenarios themselves, so a new one is covered the
    moment it is written rather than when someone remembers to list it here.
    """
    assert command_walkthrough.main(["--scenario", scenario] + FAST) == 0


def test_an_unknown_scenario_is_refused() -> None:
    """A typo on the command line is reported rather than ignored."""
    assert command_walkthrough.main(["--scenario", "nope"] + FAST) == 1


def test_motion_scenarios_are_held_back_from_a_real_robot(io_map: IoMap) -> None:
    """Against a robot, the scenarios that move it need an explicit opt-in."""
    scenarios = command_walkthrough.available_scenarios(io_map)
    runnable, skipped = command_walkthrough.select_scenarios(
        scenarios, "all", on_robot=True, allow_motion=False
    )
    names = {scenario.name for scenario in runnable}
    moving = {name for name, scenario in scenarios.items() if scenario.needs_motion}
    assert moving and not (moving & names)
    assert moving <= {name for name, _ in skipped}

    runnable, _ = command_walkthrough.select_scenarios(
        scenarios, "all", on_robot=True, allow_motion=True
    )
    assert moving <= {scenario.name for scenario in runnable}


def test_the_simulator_scenario_is_held_back_from_a_real_robot(io_map: IoMap) -> None:
    """A real robot will not fault on request, so that scenario is skipped."""
    scenarios = command_walkthrough.available_scenarios(io_map)
    _, skipped = command_walkthrough.select_scenarios(
        scenarios, "clear-fault", on_robot=True, allow_motion=True
    )
    assert [name for name, _ in skipped] == ["clear-fault"]


# ----------------------------------------------------------------------
# Dashboard
# ----------------------------------------------------------------------
class Dashboard:
    """A dashboard served on an ephemeral port, for one test.

    Args:
        robot: Facade to sample.
        commands: Commands the page may offer.
    """

    def __init__(self, robot: FieldbusRobot, commands: tuple) -> None:
        self.sampler = tag_dashboard.TagSampler(robot, poll_interval_s=0.02)
        runner = tag_dashboard.CommandRunner(robot, commands, (0.0,) * 6) if commands else None
        handler = tag_dashboard.make_handler(
            tag_dashboard.build_layout_document(robot.io_map, commands),
            self.sampler,
            str(tag_dashboard.__file__).replace("tag_dashboard.py", "tag_dashboard.html"),
            runner,
        )
        self.server = tag_dashboard.ThreadingServer(("127.0.0.1", 0), handler)
        self.port = self.server.server_address[1]
        self.sampler.start()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def url(self, path: str) -> str:
        """Return an absolute URL for a path.

        Args:
            path: Path to request.

        Returns:
            The URL.
        """
        return "http://127.0.0.1:{}{}".format(self.port, path)

    def close(self) -> None:
        """Stop the server and the sampler."""
        self.server.shutdown()
        self.server.server_close()
        self.sampler.stop()


@pytest.fixture
def dashboard(robot: FieldbusRobot) -> Dashboard:
    """Serve a read-only dashboard for one test."""
    served = Dashboard(robot, ())
    try:
        yield served
    finally:
        served.close()


@pytest.fixture
def commanding_dashboard(robot: FieldbusRobot) -> Dashboard:
    """Serve a dashboard with the safe commands enabled."""
    served = Dashboard(robot, tag_dashboard.SAFE_COMMANDS)
    try:
        yield served
    finally:
        served.close()


def test_the_page_and_the_layout_are_served(dashboard: Dashboard) -> None:
    """The page loads and the layout describes both assemblies."""
    with urllib.request.urlopen(dashboard.url("/"), timeout=5) as reply:
        assert b"<title>Cyclic tags</title>" in reply.read()
    with urllib.request.urlopen(dashboard.url("/layout"), timeout=5) as reply:
        layout = json.loads(reply.read())
    assert layout["input"]["sizeBytes"] == 252
    assert layout["output"]["sizeBytes"] == 60
    assert len(layout["input"]["fields"]) == 91


def test_every_tag_in_the_layout_carries_a_description(dashboard: Dashboard) -> None:
    """The page can show a tooltip on every tag, which is the training point."""
    with urllib.request.urlopen(dashboard.url("/layout"), timeout=5) as reply:
        layout = json.loads(reply.read())
    for assembly in ("input", "output"):
        for field in layout[assembly]["fields"]:
            assert field["description"], field["name"]


def test_a_read_only_dashboard_has_no_command_endpoint(dashboard: Dashboard) -> None:
    """Read only means the route does not exist, not that a button is hidden."""
    with urllib.request.urlopen(dashboard.url("/layout"), timeout=5) as reply:
        assert json.loads(reply.read())["commands"] == []

    request = urllib.request.Request(
        dashboard.url("/command"),
        data=json.dumps({"command": "ActivateRobot"}).encode(),
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request, timeout=5)
    assert error.value.code == 404


def test_the_stream_reports_advancing_samples(dashboard: Dashboard) -> None:
    """Two consecutive events carry a growing sequence and complete images."""
    events = []
    with urllib.request.urlopen(dashboard.url("/stream"), timeout=5) as stream:
        buffer = b""
        while len(events) < 2:
            buffer += stream.read(1)
            if buffer.endswith(b"\n\n"):
                for line in buffer.decode().splitlines():
                    if line.startswith("data: "):
                        events.append(json.loads(line[6:]))
                buffer = b""
    assert events[1]["sequence"] > events[0]["sequence"]
    assert len(events[0]["input"]) == 252 * 2
    assert len(events[0]["outputValues"]) == 26
    assert events[0]["connected"] is True


def test_an_enabled_command_runs(commanding_dashboard: Dashboard) -> None:
    """With commands enabled the endpoint exists and drives the robot."""
    request = urllib.request.Request(
        commanding_dashboard.url("/command"),
        data=json.dumps({"command": "ActivateRobot"}).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=10) as reply:
        assert json.loads(reply.read())["ok"] is True


def test_a_command_outside_the_allowed_set_is_refused(
    commanding_dashboard: Dashboard,
) -> None:
    """Enabling the safe commands does not enable the ones that move the robot."""
    request = urllib.request.Request(
        commanding_dashboard.url("/command"),
        data=json.dumps({"command": "MoveJoints"}).encode(),
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=5) as reply:
        answer = json.loads(reply.read())
    assert answer["ok"] is False
    assert "not enabled" in answer["message"]


def test_motion_commands_need_their_own_flag() -> None:
    """``--allow-commands`` alone never offers Home or MoveJoints."""
    parser = tag_dashboard.build_parser()
    safe = tag_dashboard.allowed_commands(parser.parse_args(["--allow-commands"]))
    assert "Home" not in safe and "MoveJoints" not in safe
    assert tag_dashboard.allowed_commands(parser.parse_args([])) == ()
    moving = tag_dashboard.allowed_commands(parser.parse_args(["--allow-motion"]))
    assert "Home" in moving and "MoveJoints" in moving


def test_the_brand_typeface_is_served_beside_the_page(dashboard: "Dashboard") -> None:
    """The page loads Archivo from this server, not from the internet.

    A training session often runs on a cell network with no route out, so the
    typeface is self-hosted; if this route breaks the page silently falls back
    to the system font.
    """
    with urllib.request.urlopen(dashboard.url("/fonts/Archivo-Variable.woff2"), timeout=5) as reply:
        assert reply.status == 200
        assert reply.headers["Content-Type"] == "font/woff2"
        assert reply.read(4) == b"wOF2"


def test_the_font_licence_is_served_with_it() -> None:
    """The OFL requires the licence to travel with the font."""
    licence = os.path.join(os.path.dirname(tag_dashboard.__file__), "fonts", "Archivo-OFL.txt")
    assert os.path.exists(licence), "the vendored font must keep its licence"


def test_only_whitelisted_files_are_served(dashboard: "Dashboard") -> None:
    """Nothing but the page and its font leaves this server.

    The dashboard is pointed at a robot on a customer network, so it serves a
    fixed list of paths rather than a directory: there is no traversal to find.
    """
    for path in (
        "/fonts/../tag_dashboard.py",
        "/fonts/",
        "/../README.md",
        "/tag_dashboard.py",
    ):
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(dashboard.url(path), timeout=5)
        assert error.value.code == 404, path
