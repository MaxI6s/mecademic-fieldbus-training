"""A live browser view of both cyclic process images.

Serves one page showing what the robot is publishing right now: the 252-byte
input image and the 60-byte output image as annotated hex, every decoded tag
with its unit and its description, and which bytes moved since the last sample.

Everything the browser knows about the layout comes from one ``/layout``
request, which is built from the library's I/O map.  No offset is written into
the page, so the dashboard follows the specification wherever it goes.

.. code-block:: shell

    python -m training.tag_dashboard                       # simulator, read only
    python -m training.tag_dashboard --allow-commands       # with a button bar
    python -m training.tag_dashboard --address 192.168.0.100 --allow-motion

**"Read only" has a precise meaning here.** A scanner has to produce something
or the connection dies, so connecting publishes an all-zero output image once.
What read-only guarantees is that nothing changes it afterwards: the
``/command`` route is not registered at all, so there is no button bar and no
endpoint to call.

Two things the page states on purpose, because they are the misunderstandings a
training session otherwise installs for good: the dashboard samples at 10 Hz
while the connection runs at a 10 ms RPI, so it does **not** show every frame;
and the bytes the vendor file maps no field to are greyed rather than hidden,
because an integrator has to allocate them too.
"""

import argparse
import dataclasses
import http.server
import json
import os
import queue
import socketserver
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from mecademic_fieldbus import FieldbusError, FieldbusRobot, get_io_map
from mecademic_fieldbus.io_map import INPUT_ASSEMBLY, OUTPUT_ASSEMBLY, IoMap

from .assembly_view import AssemblyView, format_value, group_of
from .backend import RecordingTransport, build_transport, describe_target
from .command_walkthrough import (
    SCENARIO_GROUPS,
    Walkthrough,
    available_scenarios,
    describe_scenarios,
    prepare,
)

__all__ = [
    "Snapshot",
    "TagSampler",
    "CommandRunner",
    "ScenarioRunner",
    "ScenarioState",
    "build_layout_document",
    "allowed_scenarios",
    "allowed_commands",
    "build_parser",
    "main",
]

#: How often the sampler reads the process images.
DEFAULT_POLL_INTERVAL_S = 0.1

#: Port the dashboard listens on.
DEFAULT_PORT = 8008

#: Commands that change the robot's state without initiating any motion.
SAFE_COMMANDS: Tuple[str, ...] = (
    "ActivateRobot",
    "DeactivateRobot",
    "ResetError",
    "PauseMotion",
    "ResumeMotion",
    "ClearMotion",
)

#: Commands that move the robot, offered only with ``--allow-motion``.
MOTION_COMMANDS: Tuple[str, ...] = ("Home", "MoveJoints")

#: How long an idle stream waits before sending a keepalive comment.
_KEEPALIVE_S = 15.0

#: The only files served beside the page, matched by exact path.  A whitelist
#: rather than a static directory: this server is pointed at a robot, and a
#: training tool is no place for a path traversal.
_STATIC = {
    "/fonts/Archivo-Variable.woff2": ("fonts/Archivo-Variable.woff2", "font/woff2"),
    "/fonts/Archivo-OFL.txt": ("fonts/Archivo-OFL.txt", "text/plain; charset=utf-8"),
}


@dataclasses.dataclass(frozen=True)
class Snapshot:
    """One sample of both process images.

    Attributes:
        sequence: Monotonically increasing sample number.
        input_image: Raw input assembly image.
        output_image: Raw output assembly image currently produced.
        changed_input: Offsets of the input bytes that moved since the previous
            sample.
        changed_output: Offsets of the output bytes that moved.
        input_values: Every input tag, decoded and rendered for display.
        output_values: Every output tag, decoded and rendered for display.
        connected: Whether the transport was connected when this was sampled.
        error: Description of the last sampling failure, empty when fine.
    """

    sequence: int
    input_image: bytes
    output_image: bytes
    changed_input: Tuple[int, ...]
    changed_output: Tuple[int, ...]
    input_values: Dict[str, str]
    output_values: Dict[str, str]
    connected: bool
    error: str

    def to_event(self, scenario: Optional[Dict[str, Any]] = None) -> str:
        """Render this snapshot as a Server-Sent Event.

        Both images are sent in full on every event rather than as a delta.
        At a hundred samples a second over the loopback interface the
        difference is irrelevant, and it removes a whole class of
        resynchronisation bug from the page.

        Args:
            scenario: Progress of a running scenario, when there is one.

        Returns:
            The event, terminated by a blank line.
        """
        payload = {
            "sequence": self.sequence,
            "input": self.input_image.hex(),
            "output": self.output_image.hex(),
            "changedInput": list(self.changed_input),
            "changedOutput": list(self.changed_output),
            "inputValues": self.input_values,
            "outputValues": self.output_values,
            "connected": self.connected,
            "error": self.error,
            "scenario": scenario,
        }
        return "data: {}\n\n".format(json.dumps(payload))


class TagSampler:
    """Poll both process images on a thread and publish each sample.

    Sampling is non-mutating: it reads the input assembly from the transport
    and the latched output image from the facade, and writes nothing.

    Args:
        robot: Connected facade to sample.
        poll_interval_s: Delay between two samples.
        output_provider: Where to read the image being produced.  Defaults to
            the facade's own latched image; a dashboard that also runs
            scenarios passes the recording transport instead, since the facade
            is then not the only writer.
    """

    def __init__(
        self,
        robot: FieldbusRobot,
        poll_interval_s: float = DEFAULT_POLL_INTERVAL_S,
        output_provider: Optional[Callable[[], bytes]] = None,
    ) -> None:
        self._robot = robot
        self._interval = poll_interval_s
        self._output_provider = output_provider or (lambda: robot.latched_output_assembly)
        self._input_view = AssemblyView(robot.io_map, INPUT_ASSEMBLY)
        self._output_view = AssemblyView(robot.io_map, OUTPUT_ASSEMBLY)
        self._lock = threading.Lock()
        self._sinks: List["queue.Queue[Snapshot]"] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._snapshot = self._sample(0, None, None)

    @property
    def snapshot(self) -> Snapshot:
        """The most recent sample."""
        with self._lock:
            return self._snapshot

    def start(self) -> None:
        """Start sampling on a daemon thread."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, name="tag-sampler", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop sampling and wait briefly for the thread to finish."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def subscribe(self) -> "queue.Queue[Snapshot]":
        """Register a sink that will receive every later sample.

        Returns:
            A queue the caller drains.
        """
        sink: "queue.Queue[Snapshot]" = queue.Queue(maxsize=4)
        with self._lock:
            self._sinks.append(sink)
        return sink

    def unsubscribe(self, sink: "queue.Queue[Snapshot]") -> None:
        """Remove a sink.

        Args:
            sink: Queue previously returned by :meth:`subscribe`.
        """
        with self._lock:
            if sink in self._sinks:
                self._sinks.remove(sink)

    def _loop(self) -> None:
        """Sample until stopped, publishing to every sink."""
        sequence = 0
        while not self._stop.is_set():
            previous = self.snapshot
            sequence += 1
            snapshot = self._sample(sequence, previous.input_image, previous.output_image)
            with self._lock:
                self._snapshot = snapshot
                sinks = list(self._sinks)
            for sink in sinks:
                try:
                    sink.put_nowait(snapshot)
                except queue.Full:
                    # A browser that cannot keep up simply misses samples; the
                    # next one carries the whole image anyway.
                    pass
            time.sleep(self._interval)

    def _sample(
        self, sequence: int, previous_input: Optional[bytes], previous_output: Optional[bytes]
    ) -> Snapshot:
        """Take one sample.

        Args:
            sequence: Sample number.
            previous_input: Previous input image, for the change list.
            previous_output: Previous output image, for the change list.

        Returns:
            The snapshot, carrying an error message rather than raising when the
            transport is unhappy.
        """
        io_map = self._robot.io_map
        try:
            raw_input = self._robot.transport.read_input_assembly()
            raw_output = self._output_provider()
            connected, error = True, ""
        except FieldbusError as failure:
            raw_input = previous_input or io_map.empty_input_assembly()
            raw_output = previous_output or io_map.empty_output_assembly()
            connected, error = False, str(failure)

        return Snapshot(
            sequence=sequence,
            input_image=raw_input,
            output_image=raw_output,
            changed_input=self._changed(self._input_view, previous_input, raw_input),
            changed_output=self._changed(self._output_view, previous_output, raw_output),
            input_values=self._render(self._input_view, raw_input),
            output_values=self._render(self._output_view, raw_output),
            connected=connected,
            error=error,
        )

    @staticmethod
    def _changed(view: AssemblyView, previous: Optional[bytes], current: bytes) -> Tuple[int, ...]:
        """Return the offsets that moved since the previous sample.

        Args:
            view: View describing the assembly.
            previous: Previous image, or ``None`` for the first sample.
            current: Current image.

        Returns:
            The changed offsets.
        """
        if previous is None:
            return ()
        return view.changed_bytes(previous, current)

    @staticmethod
    def _render(view: AssemblyView, raw: bytes) -> Dict[str, str]:
        """Decode and render every tag of an image.

        Args:
            view: View describing the assembly.
            raw: Image to decode.

        Returns:
            A mapping of field name to display string.
        """
        values = view.decode(raw)
        return {field.name: format_value(field, values[field.name]) for field in view.fields}


def build_layout_document(
    io_map: IoMap, commands: Sequence[str], scenarios: Sequence[str] = ()
) -> Dict[str, Any]:
    """Build the one-shot description the page needs to render itself.

    Every offset the browser uses comes from here, so the page carries none of
    its own.

    Args:
        io_map: Map describing the assemblies.
        commands: Commands the page may offer, possibly empty.
        scenarios: Names of the scenarios the page may start, possibly empty.

    Returns:
        A JSON-serialisable document.
    """

    def describe(assembly: str) -> Dict[str, Any]:
        """Describe one assembly for the page.

        Args:
            assembly: Which assembly to describe.

        Returns:
            Its geometry, its fields and its byte ownership map.
        """
        view = AssemblyView(io_map, assembly)
        description = view.description
        return {
            "name": description.name,
            "instance": description.instance,
            "sizeBytes": description.size_bytes,
            "direction": description.direction,
            "runIdleHeader": description.run_idle_header,
            "byteOwners": [list(owners) for owners in view.byte_owners()],
            "unmapped": list(view.unmapped_bytes()),
            "fields": [
                {
                    "name": field.name,
                    "group": group_of(field),
                    "type": field.type,
                    "byteOffset": field.byte_offset,
                    "bitOffset": field.bit_offset if field.is_bool else None,
                    "endByteOffset": field.end_byte_offset,
                    "unit": field.unit or "",
                    "description": field.description,
                }
                for field in description.fields
            ],
        }

    connection = io_map.connection
    return {
        "version": io_map.version,
        "source": dict(io_map.source),
        "connection": {
            "path": connection.connection_path,
            "rpiMicroseconds": connection.rpi_microseconds_default,
            "vendorId": connection.vendor_id,
            "productCode": connection.product_code,
        },
        "input": describe(INPUT_ASSEMBLY),
        "output": describe(OUTPUT_ASSEMBLY),
        "commands": list(commands),
        "scenarios": describe_scenarios(io_map),
        "scenarioGroups": [
            {"name": name, "title": title, "summary": summary}
            for name, title, summary in SCENARIO_GROUPS
        ],
        "allowedScenarios": list(scenarios),
        "pollIntervalMs": int(DEFAULT_POLL_INTERVAL_S * 1000),
    }


@dataclasses.dataclass(frozen=True)
class ScenarioState:
    """Where a running scenario has got to.

    Attributes:
        name: Scenario being run, empty when idle.
        step: Index of the current step, ``-1`` when idle.
        status: ``idle``, ``running``, ``waiting``, ``done``, ``failed`` or
            ``stopped``.
        message: Why it stopped, when it did not finish cleanly.
        steps: Per-step status, parallel to the scenario's steps.
        elapsed_ms: How long the current step has been waiting for the robot.
        polls: How many times the input assembly was read while waiting.  An
            integrator sizing a PLC scan rate cares about this number.
        timings: ``(milliseconds, polls)`` per step, kept once the step is
            answered.  The live pair above belongs to the step in progress and
            is gone the moment the next one starts, which would leave a
            finished sequence showing no timings at all -- the one number a
            reader is most likely to want afterwards.
    """

    name: str = ""
    step: int = -1
    status: str = "idle"
    message: str = ""
    steps: Tuple[str, ...] = ()
    elapsed_ms: int = 0
    polls: int = 0
    timings: Tuple[Tuple[int, int], ...] = ()

    def to_json(self) -> Dict[str, Any]:
        """Render the state for the page.

        Returns:
            A JSON-serialisable dictionary.
        """
        return {
            "name": self.name,
            "step": self.step,
            "status": self.status,
            "message": self.message,
            "steps": list(self.steps),
            "elapsedMs": self.elapsed_ms,
            "polls": self.polls,
            "timings": [list(pair) for pair in self.timings],
        }


class ScenarioRunner:
    """Run a walkthrough scenario on a thread, tracking where it has got to.

    The scenarios come from :mod:`training.command_walkthrough`, so the page
    and the terminal tool show the same sequences; only the rendering differs.

    Args:
        walkthrough: Driver the steps act on.
        scenarios: Scenarios available, keyed by name.
        allowed: Names the page may start.
        control_lock: Shared with the command runner, so a button press and a
            scenario can never rewrite the output image at the same time.
        step_delay_s: Pause between steps, long enough to follow on a screen.
        settle_timeout_s: Time allowed for the robot to answer one step.
    """

    def __init__(
        self,
        walkthrough: Any,
        scenarios: Dict[str, Any],
        allowed: Sequence[str],
        control_lock: threading.Lock,
        step_delay_s: float = 1.2,
        settle_timeout_s: float = 20.0,
    ) -> None:
        self._walkthrough = walkthrough
        self._scenarios = scenarios
        self._allowed = tuple(allowed)
        self._control_lock = control_lock
        self._step_delay_s = step_delay_s
        self._settle_timeout_s = settle_timeout_s
        self._lock = threading.Lock()
        self._state = ScenarioState()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    @property
    def allowed(self) -> Tuple[str, ...]:
        """Scenario names this runner will start."""
        return self._allowed

    @property
    def state(self) -> ScenarioState:
        """Where the current run has got to."""
        with self._lock:
            return self._state

    @property
    def running(self) -> bool:
        """Whether a scenario is in progress."""
        return self._thread is not None and self._thread.is_alive()

    def start(self, name: str) -> Tuple[bool, str]:
        """Start a scenario.

        Args:
            name: Scenario to run.

        Returns:
            A tuple ``(started, message)``.
        """
        if name not in self._allowed:
            return False, "{} is not enabled".format(name)
        if self.running:
            return False, "a scenario is already running"
        scenario = self._scenarios[name]
        self._stop.clear()
        with self._lock:
            self._state = ScenarioState(
                name=name,
                step=-1,
                status="running",
                steps=tuple("pending" for _ in scenario.steps),
            )
        self._thread = threading.Thread(
            target=self._run, args=(scenario,), name="scenario", daemon=True
        )
        self._thread.start()
        return True, "{} started".format(name)

    def stop(self) -> Tuple[bool, str]:
        """Ask a running scenario to stop after the current step.

        Returns:
            A tuple ``(stopping, message)``.
        """
        if not self.running:
            return False, "no scenario is running"
        self._stop.set()
        return True, "stopping after this step"

    def _run(self, scenario: Any) -> None:
        """Execute every step, publishing progress as it goes.

        Args:
            scenario: Scenario to run.
        """
        statuses = ["pending" for _ in scenario.steps]
        timings = [(0, 0) for _ in scenario.steps]
        try:
            if scenario.requires:
                self._publish(
                    scenario.name,
                    -1,
                    "preparing",
                    "bringing the robot to: {}".format(scenario.requires),
                    statuses,
                    timings=timings,
                )
                with self._control_lock:
                    prepare(self._walkthrough, scenario.requires, self._settle_timeout_s)
            for index, step in enumerate(scenario.steps):
                if self._stop.is_set():
                    self._publish(scenario.name, index, "stopped", "stopped on request", statuses)
                    return
                statuses[index] = "running"
                self._publish(scenario.name, index, "running", "", statuses, timings=timings)
                with self._control_lock:
                    step.action(self._walkthrough)
                if step.settled is not None:
                    satisfied = self._wait(scenario.name, index, step, statuses, timings)
                    statuses[index] = "done" if satisfied else "timeout"
                    if not satisfied:
                        self._publish(
                            scenario.name,
                            index,
                            "failed",
                            "the robot did not answer step {}".format(index + 1),
                            statuses,
                            timings=timings,
                        )
                        return
                else:
                    statuses[index] = "done"
                self._publish(scenario.name, index, "running", "", statuses, timings=timings)
                if self._step_delay_s and index + 1 < len(scenario.steps):
                    time.sleep(self._step_delay_s)
            self._publish(
                scenario.name, len(scenario.steps) - 1, "done", "", statuses, timings=timings
            )
        except Exception as error:  # noqa: BLE001 - a demo must not die silently
            self._publish(scenario.name, -1, "failed", str(error), statuses, timings=timings)

    def _wait(
        self,
        name: str,
        index: int,
        step: Any,
        statuses: List[str],
        timings: List[Tuple[int, int]],
    ) -> bool:
        """Poll the robot until one step is answered.

        Args:
            name: Scenario name.
            index: Step index.
            step: Step being waited on.
            statuses: Per-step status, updated in place.
            timings: Per-step ``(milliseconds, polls)``, updated in place.

        Returns:
            ``True`` when the robot answered in time.
        """
        started = time.monotonic()
        deadline = started + self._settle_timeout_s
        polls = 0
        io_map = self._walkthrough.io_map
        while True:
            polls += 1
            raw = self._walkthrough.input_image()
            elapsed = int((time.monotonic() - started) * 1000)
            timings[index] = (elapsed, polls)
            if step.settled(io_map, raw):
                self._publish(name, index, "running", "", statuses, elapsed, polls, timings)
                return True
            if time.monotonic() >= deadline or self._stop.is_set():
                self._publish(name, index, "waiting", "", statuses, elapsed, polls, timings)
                return False
            self._publish(name, index, "waiting", "", statuses, elapsed, polls, timings)
            time.sleep(0.02)

    def _publish(
        self,
        name: str,
        index: int,
        status: str,
        message: str,
        statuses: Sequence[str],
        elapsed_ms: int = 0,
        polls: int = 0,
        timings: Sequence[Tuple[int, int]] = (),
    ) -> None:
        """Record the current progress.

        Args:
            name: Scenario name.
            index: Current step index.
            status: Overall status.
            message: Explanation, when there is one.
            statuses: Per-step status.
            elapsed_ms: Time spent waiting on the current step.
            polls: Reads of the input assembly while waiting.
            timings: Per-step time and read count, kept after each step ends.
        """
        with self._lock:
            self._state = ScenarioState(
                name=name,
                step=index,
                status=status,
                message=message,
                steps=tuple(statuses),
                elapsed_ms=elapsed_ms,
                polls=polls,
                timings=tuple(timings),
            )


class CommandRunner:
    """Execute the small set of commands the page is allowed to offer.

    Serialised behind a lock, because the facade keeps one latched output image
    and two commands rewriting it at once would interleave.

    Args:
        robot: Connected facade.
        allowed: Command names the page may call.
        demo_joint_target: Joint target used by ``MoveJoints``, fixed on the
            command line so the browser cannot invent a pose.
        control_lock: Shared with the scenario runner, so a button press and a
            scenario can never rewrite the output image at the same time.
    """

    def __init__(
        self,
        robot: FieldbusRobot,
        allowed: Sequence[str],
        demo_joint_target: Sequence[float],
        control_lock: Optional[threading.Lock] = None,
    ) -> None:
        self._robot = robot
        self._allowed = tuple(allowed)
        self._target = tuple(demo_joint_target)
        self._lock = control_lock or threading.Lock()

    @property
    def allowed(self) -> Tuple[str, ...]:
        """Commands this runner accepts."""
        return self._allowed

    def run(self, name: str) -> Tuple[bool, str]:
        """Execute one command by name.

        Args:
            name: Command name.

        Returns:
            A tuple ``(succeeded, message)``; a refusal by the library is
            reported as a message rather than raised, so the page can show a
            toast instead of the browser seeing a 500.
        """
        if name not in self._allowed:
            return False, "{} is not enabled".format(name)
        with self._lock:
            try:
                if name == "MoveJoints":
                    move_id = self._robot.MoveJoints(*self._target)
                    return True, "MoveJoints {} requested as move {}".format(
                        ", ".join("{:g}".format(value) for value in self._target), move_id
                    )
                getattr(self._robot, name)()
                return True, "{} done".format(name)
            except FieldbusError as error:
                return False, str(error)


def make_handler(
    layout: Dict[str, Any],
    sampler: TagSampler,
    page: str,
    runner: Optional[CommandRunner],
    scenarios: Optional[ScenarioRunner] = None,
) -> type:
    """Build the request handler class for one dashboard instance.

    Args:
        layout: Document served at ``/layout``.
        sampler: Sampler feeding ``/stream``.
        page: Path of the HTML file served at ``/``.
        runner: Command runner, or ``None`` to leave ``/command`` unregistered
            so that a read-only dashboard has no endpoint at all.
        scenarios: Scenario runner, or ``None`` to leave ``/scenario``
            unregistered for the same reason.

    Returns:
        A :class:`http.server.BaseHTTPRequestHandler` subclass.
    """

    class DashboardHandler(http.server.BaseHTTPRequestHandler):
        """Serve the page, the layout and the sample stream."""

        protocol_version = "HTTP/1.1"
        server_version = "mecademic-fieldbus-training"

        def do_GET(self) -> None:  # noqa: N802 - the name is http.server's
            """Route a GET request."""
            if self.path in ("/", "/index.html"):
                self._send_page()
            elif self.path == "/layout":
                self._send_json(layout)
            elif self.path == "/stream":
                self._send_stream()
            elif self.path in _STATIC:
                self._send_static(self.path)
            else:
                self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802 - the name is http.server's
            """Route a POST request."""
            if self.path == "/command" and runner is not None:
                request = self._read_json()
                if request is None:
                    return
                succeeded, message = runner.run(str(request.get("command", "")))
                self._send_json({"ok": succeeded, "message": message})
                return
            if self.path == "/scenario" and scenarios is not None:
                request = self._read_json()
                if request is None:
                    return
                if request.get("action") == "stop":
                    succeeded, message = scenarios.stop()
                else:
                    succeeded, message = scenarios.start(str(request.get("name", "")))
                self._send_json({"ok": succeeded, "message": message})
                return
            self.send_error(404)

        def _read_json(self) -> Optional[Dict[str, Any]]:
            """Read a JSON body, answering 400 when it is malformed.

            Returns:
                The decoded body, or ``None`` when an error was already sent.
            """
            length = int(self.headers.get("Content-Length") or 0)
            try:
                return json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                self.send_error(400)
                return None

        def log_message(self, fmt: str, *args: Any) -> None:
            """Stay quiet; the sample stream would drown the console."""

        def _send_static(self, path: str) -> None:
            """Serve one of the few files the page loads beside itself.

            Args:
                path: Request path, already matched against the whitelist.
            """
            name, content_type = _STATIC[path]
            try:
                with open(os.path.join(os.path.dirname(page), name), "rb") as handle:
                    body = handle.read()
            except OSError:
                self.send_error(404)
                return
            self._send_bytes(body, content_type)

        def _send_page(self) -> None:
            """Serve the HTML, read from disk so it can be edited live."""
            try:
                with open(page, "rb") as handle:
                    body = handle.read()
            except OSError:
                self.send_error(500, "cannot read {}".format(page))
                return
            self._send_bytes(body, "text/html; charset=utf-8")

        def _send_json(self, document: Dict[str, Any]) -> None:
            """Serve a JSON document.

            Args:
                document: Document to serialise.
            """
            self._send_bytes(json.dumps(document).encode("utf-8"), "application/json")

        def _send_bytes(self, body: bytes, content_type: str) -> None:
            """Serve a body with its length.

            Args:
                body: Bytes to send.
                content_type: Value of the ``Content-Type`` header.
            """
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_stream(self) -> None:
            """Stream samples as Server-Sent Events until the client leaves."""
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()

            sink = sampler.subscribe()
            try:
                self.wfile.write(self._event(sampler.snapshot).encode("utf-8"))
                self.wfile.flush()
                while True:
                    try:
                        snapshot = sink.get(timeout=_KEEPALIVE_S)
                        chunk = self._event(snapshot)
                    except queue.Empty:
                        chunk = ": keepalive\n\n"
                    self.wfile.write(chunk.encode("utf-8"))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                sampler.unsubscribe(sink)

        @staticmethod
        def _event(snapshot: Snapshot) -> str:
            """Render one snapshot, with the scenario progress alongside it.

            Args:
                snapshot: Sample to render.

            Returns:
                The Server-Sent Event.
            """
            state = scenarios.state.to_json() if scenarios is not None else None
            return snapshot.to_event(state)

    return DashboardHandler


class ThreadingServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    """An HTTP server that handles the page and the stream concurrently."""

    daemon_threads = True
    allow_reuse_address = True


def build_robot(
    args: argparse.Namespace, io_map: IoMap
) -> Tuple[FieldbusRobot, Optional[Any], RecordingTransport]:
    """Build and connect the facade the dashboard samples.

    Args:
        args: Parsed command line arguments.
        io_map: Map describing the assemblies.

    Returns:
        A tuple ``(robot, simulator, recording)``; the simulator is ``None``
        against a real robot, and the recording transport is what the sampler
        reads the produced image from.

    Raises:
        FieldbusError: If a real robot cannot be reached.
    """
    transport, simulator, note = build_transport(
        io_map,
        address=args.address,
        udp_port=args.udp_port,
        activation_time_s=args.activation_time,
        homing_time_s=args.homing_time,
    )
    if note:
        print(note)
    recording = RecordingTransport(transport, io_map.empty_output_assembly())
    robot = FieldbusRobot(recording, io_map)
    robot.Connect(args.address or "in-process")
    return robot, simulator, recording


def allowed_commands(args: argparse.Namespace) -> Tuple[str, ...]:
    """Decide which commands the page may offer.

    Args:
        args: Parsed command line arguments.

    Returns:
        The command names, empty when the dashboard is read only.
    """
    if args.allow_motion:
        return SAFE_COMMANDS + MOTION_COMMANDS
    if args.allow_commands:
        return SAFE_COMMANDS
    return ()


def allowed_scenarios(args: argparse.Namespace, io_map: IoMap) -> Tuple[str, ...]:
    """Decide which scenarios the page may start.

    The same tiers as the commands: a scenario that only changes state needs
    ``--allow-commands``, one that moves the robot needs ``--allow-motion``,
    and one that provokes a fault only runs against the simulator.

    Args:
        args: Parsed command line arguments.
        io_map: Map used to build the scenarios.

    Returns:
        The scenario names, empty when the dashboard is read only.
    """
    if not (args.allow_commands or args.allow_motion):
        return ()
    on_robot = args.address is not None
    names = []
    for scenario in available_scenarios(io_map).values():
        if scenario.needs_simulator and on_robot:
            continue
        if scenario.needs_motion and not args.allow_motion:
            continue
        names.append(scenario.name)
    return tuple(names)


def build_parser() -> argparse.ArgumentParser:
    """Build the command line parser.

    Returns:
        The configured parser.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--address",
        default=None,
        help="IPv4 address of a real robot; omit to run against the in-process simulator",
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="interface to serve on (default: %(default)s)"
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT, help="port to serve on (default: %(default)s)"
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=DEFAULT_POLL_INTERVAL_S,
        help="seconds between two samples of the process images (default: %(default)s)",
    )
    parser.add_argument(
        "--allow-commands",
        action="store_true",
        help="offer the commands that change state without moving the robot",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="also offer Home and one preconfigured MoveJoints; implies --allow-commands",
    )
    parser.add_argument(
        "--demo-joint-target",
        default="0,0,0,0,0,10",
        help="comma separated joint target the MoveJoints button sends " "(default: %(default)s)",
    )
    parser.add_argument(
        "--scenario-step-delay",
        type=float,
        default=1.2,
        help="seconds to pause between the steps of a scenario, so an audience "
        "can follow (default: %(default)s)",
    )
    parser.add_argument(
        "--activation-time",
        type=float,
        default=0.5,
        help="simulated activation duration (default: %(default)s)",
    )
    parser.add_argument(
        "--homing-time",
        type=float,
        default=1.0,
        help="simulated homing duration (default: %(default)s)",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=None,
        help="UDP port to listen on; chosen automatically, and a free one is used "
        "when the target is on this machine",
    )
    return parser


def parse_target(text: str) -> List[float]:
    """Parse the demonstration joint target.

    Args:
        text: Comma separated angles.

    Returns:
        The angles.

    Raises:
        ValueError: If the text is not a list of numbers.
    """
    return [float(part) for part in text.split(",") if part.strip()]


def main(argv: Optional[List[str]] = None) -> int:
    """Serve the dashboard until interrupted.

    Args:
        argv: Command line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        The process exit code.
    """
    args = build_parser().parse_args(argv)
    commands = allowed_commands(args)
    try:
        target = parse_target(args.demo_joint_target)
    except ValueError:
        print("--demo-joint-target must be a comma separated list of numbers", file=sys.stderr)
        return 1

    if commands and args.host not in ("127.0.0.1", "localhost", "::1"):
        print(
            "WARNING: serving on {} with commands enabled -- anyone who can reach "
            "this port can drive the robot".format(args.host),
            file=sys.stderr,
        )

    io_map = get_io_map()
    scenario_names = allowed_scenarios(args, io_map)
    try:
        robot, simulator, recording = build_robot(args, io_map)
    except FieldbusError as error:
        print("cannot connect: {}".format(error), file=sys.stderr)
        return 1

    sampler = TagSampler(robot, args.poll_interval, lambda: recording.last_output)
    control_lock = threading.Lock()
    runner = CommandRunner(robot, commands, target, control_lock) if commands else None

    scenario_runner = None
    if scenario_names:
        walkthrough = Walkthrough(recording, io_map, color=False)
        walkthrough.simulator = simulator
        scenario_runner = ScenarioRunner(
            walkthrough,
            available_scenarios(io_map),
            scenario_names,
            control_lock,
            step_delay_s=args.scenario_step_delay,
        )

    page = str(__file__).replace("tag_dashboard.py", "tag_dashboard.html")
    handler = make_handler(
        build_layout_document(io_map, commands, scenario_names),
        sampler,
        page,
        runner,
        scenario_runner,
    )

    sampler.start()
    server = ThreadingServer((args.host, args.port), handler)
    print(
        "dashboard on http://{}:{}/  --  {}, sampling every {:.0f} ms".format(
            args.host,
            server.server_address[1],
            describe_target(args.address),
            args.poll_interval * 1000,
        )
    )
    print(
        "commands: {}".format(", ".join(commands))
        if commands
        else "commands: none (read only; pass --allow-commands to enable a button bar)"
    )
    print(
        "scenarios: {}".format(", ".join(scenario_names))
        if scenario_names
        else "scenarios: none (read only)"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        server.server_close()
        sampler.stop()
        try:
            robot.StopVelocityCommand()
        except FieldbusError:
            pass
        robot.Disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
