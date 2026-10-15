"""Walk through what happens on the wire when a command is sent.

A narrated, pausable sequence that shows, for each step, the output assembly
before and after, which *named tags* moved, and what the robot answered back in
its input assembly.  Built for running in front of an audience: linear,
scrollable, and pasteable into a handout.

The steps deliberately drive the **output image directly** -- set a bit, produce
the image, watch the input image -- instead of calling the facade's
``ActivateRobot()`` and friends.  Those are convenient but they hide the
handshake, which is the one thing a PLC programmer has to reimplement.  What you
see here is what their ladder or structured text has to do.

Thirteen sequences, one per command, in three groups -- basic control, motion
and data; ``--list-scenarios`` prints them.  Every sequence releases the control
bits it set once the robot has confirmed them, so each one leaves the output
image clean for the next.

Runs with no hardware and no sockets at all over
:class:`~mock_robot.loopback.LoopbackTransport`, so there is nothing to fail
live:

.. code-block:: shell

    python -m training.command_walkthrough --list-scenarios
    python -m training.command_walkthrough --mock --scenario movejoints
    python -m training.command_walkthrough --mock --scenario all --auto --step-delay 1

Against a real robot the scenarios that move it refuse to start without
``--allow-motion``:

.. code-block:: shell

    python -m training.command_walkthrough --address 192.168.0.100 --scenario activate
"""

import argparse
import dataclasses
import sys
import textwrap
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

from mecademic_fieldbus import FieldbusError, get_io_map
from mecademic_fieldbus.io_map import INPUT_ASSEMBLY, OUTPUT_ASSEMBLY, IoMap
from mecademic_fieldbus.robot_classes import JOINT_COUNT
from mecademic_fieldbus.transports.base import FieldbusTransport

from .assembly_view import AssemblyView, format_diff, format_field_table, format_value
from .backend import build_transport, describe_target

__all__ = [
    "Step",
    "Scenario",
    "Walkthrough",
    "SCENARIO_GROUPS",
    "available_scenarios",
    "prepare",
    "describe_scenarios",
    "select_scenarios",
    "build_parser",
    "main",
]

#: Default time allowed for the robot to settle after a step.
DEFAULT_SETTLE_TIMEOUT_S = 15.0

#: Delay between two reads of the input assembly while waiting for a step.
POLL_INTERVAL_S = 0.02

#: Joint the demonstration moves, zero-based: the last one, whose rotation
#: displaces the tool least.
DEMO_JOINT = JOINT_COUNT - 1

#: How far that joint moves, in degrees.
DEMO_AMPLITUDE_DEG = 10.0

#: Joint velocity held by the velocity-mode scenario, in degrees per second.
DEMO_VELOCITY_DEG_S = 20.0

#: Offline program asked for by the call-program sequence.  The simulator
#: reports the number without executing anything: nothing is stored in it.
DEMO_PROGRAM_ID = 1

#: Robot variable written and read back by the variable sequences.  Cyclic
#: identifiers from 10000 to 19999 address variables rather than commands.
DEMO_VARIABLE_ID = 10001

#: Fields a live robot changes on every single frame.  Listing them in every
#: diff would bury the one change that matters, so they are filtered out and
#: counted instead.
FREE_RUNNING_FIELDS = ("Timestamp_Second", "Timestamp_Microsecond", "DynamicDataCycles")

_BOLD = "\033[1m"
_DIM = "\033[2m"
_RESET = "\033[0m"

#: Predicate deciding when a step has taken effect.
Settled = Callable[[IoMap, bytes], bool]


@dataclasses.dataclass(frozen=True)
class Step:
    """One step of a walkthrough.

    Attributes:
        title: Short heading.
        narration: What is about to happen, and why, in prose.
        action: What to do, given the walkthrough driving it.
        settled: When to consider the robot has answered.  ``None`` means do
            not wait at all, which is what an immediate bit change wants.
        teaches: The one sentence to leave the audience with.
        sets: Output tags this step writes.  Declarative, so that a display can
            show the request side of the handshake without running anything.
        waits: Input tags that carry the robot's answer.  What ``settled``
            actually decides on, named so a display can show it changing.
        check: How a display labels the wait, for instance
            ``wait for RobotStatus_Activated``.  Empty when nothing is
            awaited.
    """

    title: str
    narration: str
    action: Callable[["Walkthrough"], None]
    settled: Optional[Settled] = None
    teaches: str = ""
    sets: Tuple[str, ...] = ()
    waits: Tuple[str, ...] = ()
    check: str = ""


@dataclasses.dataclass(frozen=True)
class Scenario:
    """A named sequence of steps.

    Attributes:
        name: Identifier passed to ``--scenario``.
        summary: One line shown by ``--list-scenarios``.
        steps: The steps, in order.
        process: A short description of the whole handshake, shown once at the
            top of a display instead of a paragraph against every step.  The
            page renders the steps themselves as a diagram, so this is where
            the explanation lives; the terminal prints it, the page does not.
        kind: How the command is triggered, which picks the glyph a display
            draws: ``level``, ``edge``, ``handshake``, ``held`` or ``slot``.
        mechanic: A few words naming that trigger.
        confirms: The tag that confirms the command took effect.
        watch: The one trap, in a single short line.
        group: Which family this belongs to, for grouping a display.
        title: Human readable name; the identifier is used when empty.
        requires: State the robot must be in before the steps make sense,
            ``"activated"`` or ``"homed"``.  Reaching it is preparation, not
            part of the sequence, so it is done before the first step and not
            shown as one.
        needs_motion: Whether running this moves the robot.
        needs_simulator: Whether this needs the simulator rather than a robot,
            because it provokes a fault the robot will not produce on request.
    """

    name: str
    summary: str
    steps: Tuple[Step, ...]
    process: str = ""
    kind: str = "level"
    mechanic: str = ""
    confirms: str = ""
    watch: str = ""
    group: str = "basic"
    title: str = ""
    requires: str = ""
    needs_motion: bool = False
    needs_simulator: bool = False

    @property
    def label(self) -> str:
        """Human readable name, falling back to the identifier."""
        return self.title or self.name


class Walkthrough:
    """Drive the output assembly step by step and narrate what moves.

    Args:
        transport: Connected transport carrying the assemblies.
        io_map: Map describing them.
        color: Whether to use ANSI escapes.
        settle_timeout_s: Time allowed for the robot to answer a step.
    """

    def __init__(
        self,
        transport: FieldbusTransport,
        io_map: IoMap,
        color: bool = True,
        settle_timeout_s: float = DEFAULT_SETTLE_TIMEOUT_S,
    ) -> None:
        self._transport = transport
        self._io_map = io_map
        self._color = color
        self._settle_timeout_s = settle_timeout_s
        self._output = io_map.empty_output_assembly()
        self._output_view = AssemblyView(io_map, OUTPUT_ASSEMBLY)
        self._input_view = AssemblyView(io_map, INPUT_ASSEMBLY)
        self.simulator = None  # type: object
        self._move_id = 0

    # ------------------------------------------------------------------
    # What a PLC does: set fields in the output image, then produce it
    # ------------------------------------------------------------------
    @property
    def io_map(self) -> IoMap:
        """Map describing the assemblies."""
        return self._io_map

    @property
    def output_image(self) -> bytes:
        """The output assembly image currently produced."""
        return self._output

    def input_image(self) -> bytes:
        """Return the latest input assembly image.

        Returns:
            The raw image the robot is producing.
        """
        return self._transport.read_input_assembly()

    def produce(self) -> None:
        """Hand the latched output image to the transport."""
        self._transport.write_output_assembly(self._output)

    def set_robot_control(self, **changes: bool) -> None:
        """Change robot control bits and produce the image.

        Args:
            **changes: Fields of
                :class:`~mecademic_fieldbus.robot_classes.RobotControl`.
        """
        control = self._io_map.decode_robot_control(self._output)
        self._output = self._io_map.encode_robot_control(
            dataclasses.replace(control, **changes), self._output
        )
        self.produce()

    def set_motion_control(self, **changes: object) -> None:
        """Change motion control bits and produce the image.

        Args:
            **changes: Fields of
                :class:`~mecademic_fieldbus.robot_classes.MotionControl`.
        """
        control = self._io_map.decode_motion_control(self._output)
        self._output = self._io_map.encode_motion_control(
            dataclasses.replace(control, **changes), self._output
        )
        self.produce()

    def set_motion_command(self, name: str, *arguments: float) -> None:
        """Write a motion command and its arguments, without triggering it.

        Nothing happens until the move id changes, which is the point: the
        arguments are staged first, so the robot can never see half of one
        command and half of the next.

        Args:
            name: Command name, looked up in the specification.
            *arguments: Up to six arguments.
        """
        from mecademic_fieldbus.robot_classes import MotionCommand

        command = MotionCommand.build(self._io_map.motion_command_id(name), arguments)
        self._output = self._io_map.encode_motion_command(command, self._output)
        self.produce()

    def set_motion_command_id(self, command_id: int, *arguments: float) -> None:
        """Write a raw command identifier and its arguments, without triggering it.

        The same field as :meth:`set_motion_command`, addressed by number
        instead of by name.  The variable sequences need it, because
        identifiers from 10000 to 19999 address robot variables and so have no
        entry in the command table.

        Args:
            command_id: Identifier to write into ``MotionCommand``.
            *arguments: Up to six arguments.
        """
        from mecademic_fieldbus.robot_classes import MotionCommand

        command = MotionCommand.build(command_id, arguments)
        self._output = self._io_map.encode_motion_command(command, self._output)
        self.produce()

    def set_dynamic_config(self, *types: int) -> None:
        """Request dynamic data types, one per slot, and produce the image.

        Args:
            *types: Type identifier for each slot from zero up, short of the
                four slots to leave the rest untouched.
        """
        self._output = self._io_map.encode_dynamic_config(types, self._output)
        self.produce()

    def set_dynamic_slot(self, slot: int, type_id: int) -> None:
        """Request one dynamic data type in one slot, leaving the others as they are.

        Args:
            slot: Slot to configure, from zero.
            type_id: Type identifier, or a variable's cyclic id.
        """
        types = list(self._io_map.decode_dynamic_config(self._output))
        types[slot] = type_id
        self._output = self._io_map.encode_dynamic_config(tuple(types), self._output)
        self.produce()

    def next_move_id(self) -> int:
        """Return the next move identifier to announce.

        Returns:
            An identifier that differs from the one before it.
        """
        self._move_id += 1
        return self._move_id

    # ------------------------------------------------------------------
    # Running a scenario
    # ------------------------------------------------------------------
    def run(self, scenario: Scenario, interactive: bool, step_delay_s: float) -> int:
        """Run every step of a scenario, narrating each one.

        Args:
            scenario: Scenario to run.
            interactive: Whether to wait for Enter between steps.
            step_delay_s: Delay between steps when not interactive.

        Returns:
            ``0`` when every step completed, ``1`` when one timed out or the
            robot reported an error.
        """
        self._heading("scenario: {}".format(scenario.label))
        print(self._wrap(scenario.process or scenario.summary))
        print()

        if scenario.requires:
            try:
                done = prepare(self, scenario.requires)
            except (FieldbusError, TimeoutError) as error:
                print("  could not prepare the robot: {}".format(error))
                return 1
            print(self._label("preparation -- not part of the sequence"))
            print("  {}, so the sequence below starts from a known state".format(done))
            print()

        for number, step in enumerate(scenario.steps, start=1):
            self._heading("step {}/{}: {}".format(number, len(scenario.steps), step.title))
            print(self._wrap(step.narration))
            print()

            before_output = self._output
            before_input = self.input_image()
            try:
                step.action(self)
            except FieldbusError as error:
                print("  the library refused the step: {}".format(error))
                return 1

            answered, polls, elapsed = self._settle(step.settled)

            print(self._label("output assembly -- what we sent"))
            print(format_diff(self._output_view, before_output, self._output, color=self._color))
            print()
            print(self._label("input assembly -- what the robot answered"))
            print(
                format_diff(
                    self._input_view,
                    before_input,
                    self.input_image(),
                    color=self._color,
                    ignore=FREE_RUNNING_FIELDS,
                )
            )
            if step.settled is not None:
                print()
                if answered:
                    print("  settled after {} poll(s), {:.0f} ms".format(polls, elapsed * 1000.0))
                else:
                    print(
                        "  the robot did not answer within {:.0f} s, after {} poll(s)".format(
                            self._settle_timeout_s, polls
                        )
                    )
                    print(self._label("sequence failed at this step"))
                    return 1
            if step.teaches:
                print()
                print(self._takeaway(step.teaches))
            print()

            if interactive:
                try:
                    input("  press Enter for the next step ... ")
                except EOFError:
                    interactive = False
                print()
            elif step_delay_s:
                time.sleep(step_delay_s)

        self._heading("scenario complete: {}".format(scenario.label))
        return 0

    def show_state(self, names: Sequence[str]) -> None:
        """Print a few decoded input fields, for context.

        Args:
            names: Field names to show.
        """
        print(self._label("robot state"))
        print(format_field_table(self._input_view, self.input_image(), names, color=self._color))

    def _settle(self, settled: Optional[Settled]) -> Tuple[bool, int, float]:
        """Poll the input assembly until a step has taken effect.

        Args:
            settled: Condition to wait for, or ``None`` to return at once.

        Returns:
            A tuple ``(settled, polls, seconds elapsed)``.  The first element
            is false when the robot never gave the answer the step waits for,
            which fails the sequence: in front of an audience a step that was
            never acknowledged must be loud, not merely annotated.
        """
        if settled is None:
            return True, 0, 0.0
        started = time.monotonic()
        deadline = started + self._settle_timeout_s
        polls = 0
        while True:
            polls += 1
            raw = self.input_image()
            if settled(self._io_map, raw):
                return True, polls, time.monotonic() - started
            if time.monotonic() >= deadline:
                return False, polls, time.monotonic() - started
            time.sleep(POLL_INTERVAL_S)

    # ------------------------------------------------------------------
    # Presentation
    # ------------------------------------------------------------------
    def _heading(self, text: str) -> None:
        """Print a section heading.

        Args:
            text: Heading text.
        """
        rule = "=" * min(len(text), 78)
        print(_BOLD + text + _RESET if self._color else text)
        print(_DIM + rule + _RESET if self._color else rule)

    def _label(self, text: str) -> str:
        """Render a sub-heading.

        Args:
            text: Label text.

        Returns:
            The label, emphasised when colour is on.
        """
        return (_BOLD + text + _RESET) if self._color else text

    def _takeaway(self, text: str) -> str:
        """Render the one sentence to remember from a step.

        Args:
            text: Takeaway text.

        Returns:
            The sentence, indented and marked.
        """
        wrapped = textwrap.fill(text, 76, initial_indent="  -> ", subsequent_indent="     ")
        return (_BOLD + wrapped + _RESET) if self._color else wrapped

    @staticmethod
    def _wrap(text: str) -> str:
        """Wrap prose for a terminal.

        Args:
            text: Text to wrap.

        Returns:
            The wrapped text, indented.
        """
        return textwrap.fill(
            " ".join(text.split()), 78, initial_indent="  ", subsequent_indent="  "
        )


# ----------------------------------------------------------------------
# Settle predicates
# ----------------------------------------------------------------------
def _activated(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports itself powered and no longer transitioning."""
    status = io_map.decode_status(raw)
    return status.activated and not status.busy


def _deactivated(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports itself powered down."""
    status = io_map.decode_status(raw)
    return not status.activated and not status.busy


def _homed(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports itself homed and ready."""
    return io_map.decode_status(raw).is_ready


def _in_error(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports a non-zero error code."""
    return io_map.decode_status(raw).error_status


def _no_error(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports no error."""
    return not io_map.decode_status(raw).error_status


def _paused(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports its motion queue paused."""
    return io_map.decode_motion_status(raw).paused


def _running(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports its motion queue not paused."""
    return not io_map.decode_motion_status(raw).paused


def _idle(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot has stopped moving with an empty queue."""
    return io_map.decode_motion_status(raw).end_of_block


def _moving(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports itself in motion."""
    return not io_map.decode_motion_status(raw).end_of_movement


def _cleared(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports its motion queue emptied."""
    return io_map.decode_motion_status(raw).cleared


def _program_running(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot reports an offline program as running."""
    return io_map.decode_motion_status(raw).offline_program_id != 0


def _joint_vector(value: float) -> Tuple[float, ...]:
    """Build six joint arguments with only the demonstration joint set.

    Args:
        value: Value for the demonstration joint, an angle or a velocity.

    Returns:
        One value per joint.
    """
    return tuple(value if index == DEMO_JOINT else 0.0 for index in range(JOINT_COUNT))


# ----------------------------------------------------------------------
# Preparation: reaching the state a sequence assumes
# ----------------------------------------------------------------------
def prepare(walkthrough: "Walkthrough", requires: str, timeout_s: float = 20.0) -> str:
    """Bring the robot to the state a sequence assumes, before it starts.

    Activating and homing are sequences of their own; repeating them inside
    every other one would bury the command each is meant to show.  So they run
    first, quietly, and the display says what was done rather than walking it.

    Args:
        walkthrough: Driver to act through.
        requires: ``"activated"``, ``"homed"``, or ``""`` for nothing.
        timeout_s: Time allowed to reach the state.

    Returns:
        A line describing what was done, empty when nothing was needed.

    Raises:
        TimeoutError: If the robot does not reach the state in time.
    """
    if not requires:
        return ""
    io_map = walkthrough.io_map
    done = []
    if not io_map.decode_status(walkthrough.input_image()).activated:
        walkthrough.set_robot_control(activate=True, deactivate=False)
        _await(walkthrough, _activated, timeout_s, "activate")
        # A request, released once confirmed -- the same as the sequence teaches.
        walkthrough.set_robot_control(activate=False)
        done.append("activated")
    if requires == "homed" and not io_map.decode_status(walkthrough.input_image()).homed:
        walkthrough.set_robot_control(home=True)
        _await(walkthrough, _homed, timeout_s, "home")
        walkthrough.set_robot_control(home=False)
        done.append("homed")
    if requires == "homed" and io_map.decode_motion_status(walkthrough.input_image()).paused:
        # The pause and clear-motion sequences leave the robot paused, which
        # would hold every move here indefinitely.
        walkthrough.set_motion_control(pause=False, clear_move=False, resume_motion=False)
        walkthrough.set_motion_control(resume_motion=True)
        _await(walkthrough, _running, timeout_s, "resume")
        walkthrough.set_motion_control(resume_motion=False)
        done.append("resumed")
    if not done:
        return "robot already {}".format(requires)
    return "robot " + " and ".join(done)


def _await(walkthrough: "Walkthrough", settled: Settled, timeout_s: float, what: str) -> None:
    """Poll until a condition holds.

    Args:
        walkthrough: Driver to read through.
        settled: Condition to wait for.
        timeout_s: Time allowed.
        what: What is being waited on, for the error message.

    Raises:
        TimeoutError: If the condition never holds.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if settled(walkthrough.io_map, walkthrough.input_image()):
            return
        time.sleep(POLL_INTERVAL_S)
    raise TimeoutError("the robot did not {} within {:.0f} s".format(what, timeout_s))


# ----------------------------------------------------------------------
# Building blocks shared by the sequences
# ----------------------------------------------------------------------
def _acknowledged(io_map: IoMap, raw: bytes) -> bool:
    """Whether the robot echoes a move id back."""
    return io_map.decode_motion_status(raw).move_id != 0


def _hand_over(command: Union[str, int], *arguments: float) -> Callable[["Walkthrough"], None]:
    """Build the action that hands a command over to the robot.

    The identifier and the arguments are staged first and only then is the
    move id changed with the setpoint set, so the robot can never see half of
    one command and half of the next.

    Args:
        command: Command name from the table, or a raw identifier such as a
            variable's cyclic id, which has no name.
        *arguments: Up to six arguments.

    Returns:
        The action.
    """

    def action(walk: "Walkthrough") -> None:
        if isinstance(command, str):
            walk.set_motion_command(command, *arguments)
        else:
            walk.set_motion_command_id(command, *arguments)
        walk.set_motion_control(move_id=walk.next_move_id(), setpoint=True)

    return action


def _reset_setpoint(
    waits: Tuple[str, ...] = (), settled: Optional[Settled] = None, check: str = ""
) -> Step:
    """Build the step that puts MotionControl_Setpoint back down.

    Setpoint says "this output carries a command".  Once the robot has taken
    the command it no longer does; the queued move carries on regardless.

    Args:
        waits: What to wait for afterwards, if anything.
        settled: The matching condition.
        check: How a display labels the wait.

    Returns:
        The step.
    """
    return Step(
        title="reset MotionControl_Setpoint",
        check=check,
        narration="""
            The robot has taken the command, so the setpoint goes back down:
            the output no longer carries a command. Whatever was queued
            carries on.
        """,
        action=lambda walk: walk.set_motion_control(setpoint=False),
        settled=settled,
        sets=("MotionControl_Setpoint",),
        waits=waits,
    )


# ----------------------------------------------------------------------
# Basic: the robot control bits
# ----------------------------------------------------------------------
def _activate_scenario() -> Scenario:
    """Build the activation sequence.

    Returns:
        The sequence.
    """
    return Scenario(
        name="activate",
        kind="level",
        mechanic="Level request, released once confirmed",
        confirms="RobotStatus_Activated",
        watch="Left set, the robot reactivates itself whenever it drops.",
        process="""
    One bit, at byte 0.1. The scanner sets RobotControl_Activate and keeps
    producing that image until the robot reports RobotStatus_Activated, then
    releases it. The bit is a request, not a state to hold: while it is set
    the vendor says the robot activates itself, so a robot brought down by a
    fault reset would come straight back up. Released, the robot stays
    activated -- only RobotControl_Deactivate powers it down.
        """,
        title="Activate",
        group="basic",
        summary="Power the motors up: set the request, wait for the robot to "
        "confirm it, release it.",
        steps=(
            Step(
                title="set RobotControl_Activate",
                check="wait for RobotStatus_Activated = True",
                narration="""
                    One bit, at byte 0.1. No message and no acknowledgement to
                    read: the scanner publishes an image with that bit set, and
                    the answer is a bit in the input image.
                """,
                action=lambda walk: walk.set_robot_control(activate=True, deactivate=False),
                settled=_activated,
                sets=("RobotControl_Activate",),
                waits=("RobotStatus_Activated",),
                teaches="Never time a state change: wait for the bit that confirms it.",
            ),
            Step(
                title="reset RobotControl_Activate",
                narration="""
                    Confirmed, so the request goes back down. The robot stays
                    activated: releasing a request does not undo it.
                """,
                action=lambda walk: walk.set_robot_control(activate=False),
                sets=("RobotControl_Activate",),
                teaches="Held, the request keeps acting. Release it once confirmed.",
            ),
        ),
    )


def _deactivate_scenario() -> Scenario:
    """Build the deactivation sequence.

    Returns:
        The sequence.
    """
    return Scenario(
        name="deactivate",
        kind="level",
        mechanic="Level request, wins over Activate",
        confirms="RobotStatus_Activated → false",
        watch="Held, it blocks every later activation.",
        process="""
    Deactivate is its own bit and wins over Activate. It is set until
    RobotStatus_Activated drops, then released -- held, it would block every
    later activation. Deactivating also clears the homing reference, so the
    robot has to be homed again afterwards.
        """,
        title="Deactivate",
        group="basic",
        requires="activated",
        summary="Power the motors down: set the request, wait for Activated to "
        "drop, release it.",
        steps=(
            Step(
                title="set RobotControl_Deactivate",
                check="wait for RobotStatus_Activated = False",
                narration="""
                    Deactivate is its own bit, and the vendor documents it as
                    winning over Activate.
                """,
                action=lambda walk: walk.set_robot_control(deactivate=True),
                settled=_deactivated,
                sets=("RobotControl_Deactivate",),
                waits=("RobotStatus_Activated",),
            ),
            Step(
                title="reset RobotControl_Deactivate",
                narration="""
                    Released once confirmed: held, it wins over every later
                    activation request.
                """,
                action=lambda walk: walk.set_robot_control(deactivate=False),
                sets=("RobotControl_Deactivate",),
                teaches="Deactivating clears the homing reference too.",
            ),
        ),
    )


def _home_scenario() -> Scenario:
    """Build the homing sequence.

    Returns:
        The sequence.
    """
    return Scenario(
        name="home",
        kind="level",
        mechanic="Level request, released once confirmed",
        confirms="RobotStatus_Homed",
        watch="Nothing moves until Homed is set.",
        process="""
    The same shape as activation: RobotControl_Home is set until
    RobotStatus_Homed comes back, then released, and the robot stays homed.
    Until Homed is set the robot refuses to move, whatever you send it.
        """,
        title="Home",
        group="basic",
        requires="activated",
        needs_motion=True,
        summary="Find the home position: set the request, wait for Homed, " "release it.",
        steps=(
            Step(
                title="set RobotControl_Home",
                check="wait for RobotStatus_Homed = True",
                narration="""
                    The same shape as activation. RobotStatus_Homed is the gate
                    on every motion command.
                """,
                action=lambda walk: walk.set_robot_control(home=True),
                settled=_homed,
                sets=("RobotControl_Home",),
                waits=("RobotStatus_Homed",),
                teaches="Until Homed is set the robot refuses to move.",
            ),
            Step(
                title="reset RobotControl_Home",
                narration="Confirmed, so the request goes back down. The robot stays homed.",
                action=lambda walk: walk.set_robot_control(home=False),
                sets=("RobotControl_Home",),
            ),
        ),
    )


def _clear_fault_scenario() -> Scenario:
    """Build the fault reset sequence.

    Returns:
        The sequence.
    """

    def provoke(walk: "Walkthrough") -> None:
        """Make the simulated robot fault, which a real one will not do on cue."""
        walk.simulator.inject_error()

    return Scenario(
        name="clear-fault",
        kind="level",
        mechanic="Level request, released once cleared",
        confirms="RobotStatus_Error → 0",
        watch="There is no error bit: test the code against zero.",
        process="""
    There is no error bit. A non-zero RobotStatus_Error, a uint16 at byte 2,
    is the error. RobotControl_ResetError is set until the code drops back to
    zero, then released. On this robot clearing a fault also powers it down,
    so activation and homing have to follow.
        """,
        title="Clear fault",
        group="basic",
        requires="activated",
        needs_simulator=True,
        summary="There is no error bit: a non-zero code is the error. Set the "
        "reset, wait for zero, release it.",
        steps=(
            Step(
                title="inject a fault (simulator)",
                check="wait for RobotStatus_Error ≠ 0",
                narration="""
                    The simulator faults on request. Watch RobotStatus_Error, a
                    uint16 at byte 2, go non-zero.
                """,
                action=provoke,
                settled=_in_error,
                waits=("RobotStatus_Error",),
                teaches="Test the error code against zero; there is no error boolean.",
            ),
            Step(
                title="set RobotControl_ResetError",
                check="wait for RobotStatus_Error = 0",
                narration="Held until the code drops back to zero.",
                action=lambda walk: walk.set_robot_control(reset_error=True),
                settled=_no_error,
                sets=("RobotControl_ResetError",),
                waits=("RobotStatus_Error",),
            ),
            Step(
                title="reset RobotControl_ResetError",
                narration="""
                    Released once cleared. The robot is now deactivated:
                    activation and homing have to follow.
                """,
                action=lambda walk: walk.set_robot_control(reset_error=False),
                sets=("RobotControl_ResetError",),
            ),
        ),
    )


# ----------------------------------------------------------------------
# Motion: the motion control bits and the command handshake
# ----------------------------------------------------------------------
def _pause_scenario() -> Scenario:
    """Build the pause sequence.

    Returns:
        The sequence.
    """
    return Scenario(
        name="pause",
        kind="level",
        mechanic="Level request, released once confirmed",
        confirms="MotionStatus_Paused",
        watch="Releasing Pause does not resume. ResumeMotion does.",
        process="""
    MotionControl_Pause stops the robot without clearing its queue, confirmed
    by MotionStatus_Paused. It is released once confirmed, and the robot stays
    paused: only a rising edge on MotionControl_ResumeMotion resumes it.
        """,
        title="Pause motion",
        group="motion",
        requires="activated",
        summary="Stop the robot, keep its queue: set Pause, wait for Paused, " "release it.",
        steps=(
            Step(
                title="set MotionControl_Pause",
                check="wait for MotionStatus_Paused = True",
                narration="The robot pauses with its queue intact.",
                action=lambda walk: walk.set_motion_control(pause=True),
                settled=_paused,
                sets=("MotionControl_Pause",),
                waits=("MotionStatus_Paused",),
                teaches="Pause holds the queue; clear motion empties it.",
            ),
            Step(
                title="reset MotionControl_Pause",
                narration="Released once confirmed. The robot stays paused.",
                action=lambda walk: walk.set_motion_control(pause=False),
                sets=("MotionControl_Pause",),
                teaches="Releasing Pause is not resuming. That takes its own bit.",
            ),
        ),
    )


def _resume_scenario() -> Scenario:
    """Build the resume sequence.

    Returns:
        The sequence.
    """
    return Scenario(
        name="resume",
        kind="edge",
        mechanic="Rising edge — the only one in the assembly",
        confirms="MotionStatus_Paused → false",
        watch="Left high, the next pause cannot be undone.",
        process="""
    MotionControl_ResumeMotion acts on its rising edge only, the one bit in
    the whole assembly that does. The robot is paused first so there is
    something to resume; then Pause is released and ResumeMotion raised in the
    same image -- one transition -- and released straight after. Left high,
    the next pause becomes impossible to undo.
        """,
        title="Resume motion",
        group="motion",
        requires="activated",
        summary="The only rising-edge bit: raise ResumeMotion, wait for Paused "
        "to drop, release it.",
        steps=(
            Step(
                title="set MotionControl_Pause",
                check="wait for MotionStatus_Paused = True",
                narration="Paused first, so that resuming has something to undo.",
                action=lambda walk: walk.set_motion_control(pause=True),
                settled=_paused,
                sets=("MotionControl_Pause",),
                waits=("MotionStatus_Paused",),
            ),
            Step(
                title="reset Pause, set MotionControl_ResumeMotion",
                check="wait for MotionStatus_Paused = False",
                narration="""
                    ResumeMotion goes from 0 to 1 in this image: a rising edge,
                    which is the only thing this bit acts on.
                """,
                action=lambda walk: walk.set_motion_control(pause=False, resume_motion=True),
                settled=_running,
                sets=("MotionControl_Pause", "MotionControl_ResumeMotion"),
                waits=("MotionStatus_Paused",),
                teaches="An edge, not a level: it must go back down before it can act again.",
            ),
            Step(
                title="reset MotionControl_ResumeMotion",
                narration="Released straight away, ready for the next rising edge.",
                action=lambda walk: walk.set_motion_control(resume_motion=False),
                sets=("MotionControl_ResumeMotion",),
            ),
        ),
    )


def _clear_motion_scenario() -> Scenario:
    """Build the clear motion sequence.

    Returns:
        The sequence.
    """
    return Scenario(
        name="clear-motion",
        kind="level",
        mechanic="Level request, released once confirmed",
        confirms="MotionStatus_Cleared",
        watch="While it is set, the robot queues nothing.",
        process="""
    MotionControl_ClearMove empties the queue and pauses, confirmed by
    MotionStatus_Cleared. While the bit is set the robot accepts nothing new,
    so it is released as soon as the confirmation arrives. The robot stays
    paused; motion needs an explicit resume.
        """,
        title="Clear motion",
        group="motion",
        requires="activated",
        summary="Empty the queue: set ClearMove, wait for Cleared, release it.",
        steps=(
            Step(
                title="set MotionControl_ClearMove",
                check="wait for MotionStatus_Cleared = True",
                narration="The queue is emptied and the robot pauses.",
                action=lambda walk: walk.set_motion_control(clear_move=True),
                settled=_cleared,
                sets=("MotionControl_ClearMove",),
                waits=("MotionStatus_Cleared",),
            ),
            Step(
                title="reset MotionControl_ClearMove",
                narration="""
                    Released once confirmed; held, nothing could be queued
                    again. The robot stays paused.
                """,
                action=lambda walk: walk.set_motion_control(clear_move=False),
                sets=("MotionControl_ClearMove",),
                teaches="Hold it only until Cleared comes back.",
            ),
        ),
    )


def _send_command_scenario(io_map: IoMap) -> Scenario:
    """Build the queued command sequence.

    Args:
        io_map: Map used to resolve the command identifier.

    Returns:
        The sequence.
    """
    return Scenario(
        name="send-command",
        kind="handshake",
        mechanic="Move-id handshake",
        confirms="MotionStatus_MoveID, then EOB",
        watch="The robot acts on the change of the id, not its value.",
        process="""
    MotionCommand takes the identifier of MoveJoints ({}) and bytes 12 to 35
    the six float arguments; then MotionControl_MoveId changes with
    MotionControl_Setpoint set. The robot echoes the id in MotionStatus_MoveID,
    which means queued, and the setpoint goes back down. MotionStatus_EOB then
    means finished. The robot acts on the change of the move id, not its
    value, so reusing one it already handled discards the command silently.
        """.format(io_map.motion_command_id("MoveJoints")),
        title="Send a command",
        group="motion",
        requires="homed",
        needs_motion=True,
        summary="The move-id handshake every one of the 48 commands shares, with "
        "MoveJoints as the example.",
        steps=(
            Step(
                title="set MoveJoints, new MoveId, Setpoint",
                check="wait for MotionStatus_MoveID = MoveId",
                narration="""
                    The command and its arguments are staged, then the move id
                    changes with the setpoint set. The echo in MoveID is the
                    only acknowledgement the protocol gives.
                """,
                action=_hand_over("MoveJoints", *_joint_vector(DEMO_AMPLITUDE_DEG)),
                settled=_acknowledged,
                sets=(
                    "MotionCommand",
                    "MotionCommand_Arg6",
                    "MotionControl_MoveId",
                    "MotionControl_Setpoint",
                ),
                waits=("MotionStatus_MoveID",),
                teaches="The robot acts on the change of the move id, not its value.",
            ),
            _reset_setpoint(("MotionStatus_EOB",), _idle, "wait for MotionStatus_EOB = True"),
        ),
    )


def _velocity_scenario(io_map: IoMap) -> Scenario:
    """Build the velocity-mode sequence.

    Args:
        io_map: Map used to resolve the command identifier.

    Returns:
        The sequence.
    """

    def hold(walk: "Walkthrough") -> None:
        """Stage and hold a joint velocity with a move id of zero."""
        walk.set_motion_command("MoveJointsVel", *_joint_vector(DEMO_VELOCITY_DEG_S))
        walk.set_motion_control(move_id=0, setpoint=True)

    return Scenario(
        name="velocity",
        kind="held",
        mechanic="Held with MoveId = 0",
        confirms="MotionStatus_EOM",
        watch="Change the id instead and a copy queues every cycle.",
        process="""
    The same fields as a queued move, MoveJointsVel ({}), but the move id
    stays at zero -- which tells the robot to keep applying the command every
    cycle instead of queueing it. It moves for as long as the setpoint is set,
    and the arguments can be rewritten with no handshake at all, which is how
    an axis is jogged from a PLC. Resetting the setpoint stops it. Change the
    move id instead and the robot appends a copy of the command every cycle.
        """.format(io_map.motion_command_id("MoveJointsVel")),
        title="Velocity mode",
        group="motion",
        requires="homed",
        needs_motion=True,
        summary="The one integrators get wrong: a velocity command is held with "
        "a move id of zero, never queued.",
        steps=(
            Step(
                title="set MoveJointsVel, Setpoint, MoveId = 0",
                check="wait for MotionStatus_EOM = False",
                narration="""
                    The move id stays at zero: held, not queued. The robot
                    moves for as long as the setpoint is set.
                """,
                action=hold,
                settled=_moving,
                sets=(
                    "MotionCommand",
                    "MotionCommand_Arg6",
                    "MotionControl_Setpoint",
                    "MotionControl_MoveId",
                ),
                waits=("MotionStatus_EOM",),
                teaches="MoveId = 0 means held. MoveId changing means queued.",
            ),
            _reset_setpoint(("MotionStatus_EOB",), _idle, "wait for MotionStatus_EOB = True"),
        ),
    )


def _call_program_scenario(io_map: IoMap) -> Scenario:
    """Build the offline program sequence.

    Args:
        io_map: Map used to resolve the command identifier.

    Returns:
        The sequence.
    """
    return Scenario(
        name="call-program",
        kind="handshake",
        mechanic="Move-id handshake",
        confirms="MotionStatus_OfflineProgramId",
        watch="All 48 commands share this one handshake.",
        process="""
    StartProgram, command {}, takes the program number as its first argument
    and is handed over exactly like a move: stage it, change the move id with
    the setpoint set, release the setpoint. The robot reports the program it
    is running in MotionStatus_OfflineProgramId, and zero when none is. The
    simulator reports the number without executing anything.
        """.format(io_map.motion_command_id("StartProgram")),
        title="Call a program",
        group="motion",
        requires="homed",
        needs_motion=True,
        summary="Start an offline program with the same handshake as any command.",
        steps=(
            Step(
                title="set StartProgram {}, new MoveId, Setpoint".format(DEMO_PROGRAM_ID),
                check="wait for MotionStatus_OfflineProgramId = {}".format(DEMO_PROGRAM_ID),
                narration="The same handshake as a move; only the command differs.",
                action=_hand_over("StartProgram", float(DEMO_PROGRAM_ID)),
                settled=_program_running,
                sets=(
                    "MotionCommand",
                    "MotionCommand_Arg1",
                    "MotionControl_MoveId",
                    "MotionControl_Setpoint",
                ),
                waits=("MotionStatus_OfflineProgramId",),
            ),
            _reset_setpoint(),
        ),
    )


# ----------------------------------------------------------------------
# Data: the dynamic slots and the variables
# ----------------------------------------------------------------------
def _dynamic_data_scenario(io_map: IoMap) -> Scenario:
    """Build the dynamic data sequence.

    Args:
        io_map: Map used to resolve the type identifier.

    Returns:
        The sequence.
    """
    joint_type = io_map.dynamic_data_type_id("ActualJointPosition")
    return Scenario(
        name="dynamic-data",
        kind="slot",
        mechanic="Requested type, echoed back",
        confirms="DynamicType0",
        watch="Check the echo before trusting the values.",
        process="""
    The fixed fields carry the target position, not the measured one. Write a
    type identifier into DynamicTypeCfg0 -- {} is the actual joint position --
    and the robot echoes it back in DynamicType0 with six values alongside.
    Check the echo before reading the values: a change takes a cycle or two.
    The slot is a configuration word, not a control bit, so it stays set.
        """.format(joint_type),
        title="Dynamic data",
        group="data",
        requires="activated",
        summary="Ask for what the fixed fields do not carry: request a type, "
        "wait for the echo, read the values.",
        steps=(
            Step(
                title="set DynamicTypeCfg0 = {}".format(joint_type),
                check="wait for DynamicType0 = {}".format(joint_type),
                narration="""
                    The robot echoes the requested type, which is how a reader
                    knows the values belong to this request.
                """,
                action=lambda walk: walk.set_dynamic_slot(0, joint_type),
                settled=lambda io, raw: io.decode_dynamic_data(raw, 0).type_id == joint_type,
                sets=("DynamicTypeCfg0",),
                waits=("DynamicType0", "DynamicData0_value5"),
                teaches="Check the echoed type before trusting the values.",
            ),
        ),
    )


def _set_variable_scenario() -> Scenario:
    """Build the variable write sequence.

    Returns:
        The sequence.
    """
    return Scenario(
        name="set-variable",
        kind="handshake",
        mechanic="Move-id handshake, id 10000–19999",
        confirms="MotionStatus_MoveID",
        watch="Same field as a command; only the id range differs.",
        process="""
    No new mechanism: the same MotionCommand field and the same move-id
    handshake as any command. Only the identifier range differs -- 10000 to
    19999 addresses a robot variable, and the arguments are its value. Here
    variable {} receives 1.5, 2.5 and 3.5.
        """.format(DEMO_VARIABLE_ID),
        title="Set a variable",
        group="data",
        requires="activated",
        summary="Write a robot variable through the command field.",
        steps=(
            Step(
                title="set {} + values, new MoveId, Setpoint".format(DEMO_VARIABLE_ID),
                check="wait for MotionStatus_MoveID = MoveId",
                narration="The command handshake, addressed to a variable.",
                action=_hand_over(DEMO_VARIABLE_ID, 1.5, 2.5, 3.5),
                settled=_acknowledged,
                sets=(
                    "MotionCommand",
                    "MotionCommand_Arg1",
                    "MotionCommand_Arg2",
                    "MotionCommand_Arg3",
                    "MotionControl_MoveId",
                    "MotionControl_Setpoint",
                ),
                waits=("MotionStatus_MoveID",),
                teaches="The identifier range decides: command, program or variable.",
            ),
            _reset_setpoint(),
        ),
    )


def _get_variable_scenario() -> Scenario:
    """Build the variable read sequence.

    Returns:
        The sequence.
    """

    def request(walk: "Walkthrough") -> None:
        """Release the setpoint and point slot 1 at the variable."""
        walk.set_motion_control(setpoint=False)
        walk.set_dynamic_slot(1, DEMO_VARIABLE_ID)

    return Scenario(
        name="get-variable",
        kind="slot",
        mechanic="Slot addressed by the variable id",
        confirms="DynamicType1",
        watch="Write through a command, read through a slot.",
        process="""
    The variable is written first, as in the set-variable sequence. Then the
    setpoint is released and DynamicTypeCfg1 takes the same id, {}: the robot
    echoes it in DynamicType1 and publishes the variable's value in that slot.
        """.format(DEMO_VARIABLE_ID),
        title="Get a variable",
        group="data",
        requires="activated",
        summary="Read a robot variable back through a dynamic data slot.",
        steps=(
            Step(
                title="set {} + values, new MoveId, Setpoint".format(DEMO_VARIABLE_ID),
                check="wait for MotionStatus_MoveID = MoveId",
                narration="Written first, so there is something to read.",
                action=_hand_over(DEMO_VARIABLE_ID, 1.5, 2.5, 3.5),
                settled=_acknowledged,
                sets=(
                    "MotionCommand",
                    "MotionCommand_Arg1",
                    "MotionCommand_Arg2",
                    "MotionCommand_Arg3",
                    "MotionControl_MoveId",
                    "MotionControl_Setpoint",
                ),
                waits=("MotionStatus_MoveID",),
            ),
            Step(
                title="reset Setpoint, set DynamicTypeCfg1 = {}".format(DEMO_VARIABLE_ID),
                check="wait for DynamicType1 = {}".format(DEMO_VARIABLE_ID),
                narration="""
                    The same cyclic id, in a dynamic slot this time: the robot
                    echoes it and publishes the value.
                """,
                action=request,
                settled=lambda io, raw: io.decode_dynamic_data(raw, 1).type_id == DEMO_VARIABLE_ID,
                sets=("MotionControl_Setpoint", "DynamicTypeCfg1"),
                waits=("DynamicType1", "DynamicData1_value0"),
                teaches="Write through the command field, read through a slot.",
            ),
        ),
    )


#: Cyclic identifiers that address a robot variable rather than a command or a
#: dynamic data type.
_VARIABLE_IDS = range(10000, 20000)

#: The output words that request a dynamic data type, one per slot.
_DYNAMIC_CONFIG_FIELDS = tuple("DynamicTypeCfg{}".format(slot) for slot in range(4))


def _rehearse(io_map: IoMap, scenario: Scenario, first_move_id: int) -> List[Dict[str, Any]]:
    """Run a scenario's actions on a throwaway simulator and record what they write.

    Only the output image is read, and it depends on the actions alone, so
    nothing is waited for: this is the scenario's own code telling the page
    what it sets, rather than a second copy of that knowledge to keep in step.

    Args:
        io_map: Map describing the assemblies.
        scenario: Scenario to rehearse.
        first_move_id: Where the move-id counter starts.

    Returns:
        For each step, the value of every tag it declares it sets.
    """
    from mock_robot import LoopbackTransport, RobotSimulator

    simulator = RobotSimulator(io_map, activation_time_s=0.0, homing_time_s=0.0)
    transport = LoopbackTransport(simulator)
    transport.connect("rehearsal")
    walkthrough = Walkthrough(transport, io_map, color=False)
    walkthrough.simulator = simulator
    walkthrough._move_id = first_move_id
    written = []
    for step in scenario.steps:
        step.action(walkthrough)
        values = io_map.decode_all(OUTPUT_ASSEMBLY, walkthrough.output_image)
        written.append({name: values[name] for name in step.sets})
    return written


def describe_targets(io_map: IoMap, scenario: Scenario) -> List[List[Tuple[str, str]]]:
    """Describe the value each step writes into each tag it sets.

    The scenario is rehearsed twice with the move-id counter started at two
    different places.  A tag whose value differs between the two depends on
    the run, which in practice means the move id, and is described as
    ``previous + 1`` rather than as whatever number one rehearsal produced.
    Identifiers are annotated with what they name.

    Args:
        io_map: Map describing the assemblies.
        scenario: Scenario to describe.

    Returns:
        For each step, ``(tag, value)`` pairs in the order the step declares.
    """
    fields = {field.name: field for field in io_map.describe_fields(OUTPUT_ASSEMBLY)}
    commands = {value: name for name, value in io_map.motion_command_ids.items()}
    types = {value: name for name, value in io_map.dynamic_data_types.items()}

    def describe(name: str, value: Any, other: Any) -> str:
        field = fields[name]
        if value != other:
            return "previous + 1"
        if field.is_bool:
            return "True" if value else "False"
        text = format_value(field, value)
        label = ""
        if name == "MotionCommand":
            label = commands.get(int(value), "")
        elif name.startswith("DynamicTypeCfg"):
            label = types.get(int(value), "")
        if (
            not label
            and name in ("MotionCommand",) + _DYNAMIC_CONFIG_FIELDS
            and (int(value) in _VARIABLE_IDS)
        ):
            label = "variable"
        return "{} \u00b7 {}".format(text, label) if label else text

    first = _rehearse(io_map, scenario, 0)
    second = _rehearse(io_map, scenario, 1000)
    return [
        [(name, describe(name, value, again[name])) for name, value in step.items()]
        for step, again in zip(first, second)
    ]


def describe_scenarios(io_map: IoMap) -> List[Dict[str, Any]]:
    """Describe every scenario as plain data, for a display to render.

    The scenarios are defined once, as code, and both the terminal walkthrough
    and the dashboard read them from here.  Anything else and the two drift,
    which in a training tool means teaching two different protocols.

    Args:
        io_map: Map used to resolve command identifiers.

    Returns:
        One entry per scenario, each listing its group, its prerequisite and
        its steps with the tags they write and the tags that carry the robot's
        answer.
    """
    described = []
    for scenario in available_scenarios(io_map).values():
        targets = describe_targets(io_map, scenario)
        described.append(
            {
                "name": scenario.name,
                "label": scenario.label,
                "process": " ".join(scenario.process.split()),
                "kind": scenario.kind,
                "mechanic": scenario.mechanic,
                "confirms": scenario.confirms,
                "watch": scenario.watch,
                "group": scenario.group,
                "requires": scenario.requires,
                "summary": scenario.summary,
                "needsMotion": scenario.needs_motion,
                "needsSimulator": scenario.needs_simulator,
                "steps": [
                    {
                        "title": step.title,
                        "narration": " ".join(step.narration.split()),
                        "teaches": step.teaches,
                        "check": step.check
                        or ("wait for {}".format(step.waits[0]) if step.waits else ""),
                        "sets": list(step.sets),
                        "targets": [list(pair) for pair in target],
                        "waits": list(step.waits),
                        "blocking": step.settled is not None,
                    }
                    for step, target in zip(scenario.steps, targets)
                ],
            }
        )
    return described


def available_scenarios(io_map: IoMap) -> Dict[str, Scenario]:
    """Build every scenario, keyed by name.

    Args:
        io_map: Map used to resolve command identifiers.

    Returns:
        The scenarios, in the order they are best presented.
    """
    scenarios = (
        # Basic: the robot control bits.
        _activate_scenario(),
        _deactivate_scenario(),
        _home_scenario(),
        _clear_fault_scenario(),
        # Motion: the motion control bits and the command handshake.
        _pause_scenario(),
        _resume_scenario(),
        _clear_motion_scenario(),
        _send_command_scenario(io_map),
        _velocity_scenario(io_map),
        _call_program_scenario(io_map),
        # Data: the dynamic slots and the variables.
        _dynamic_data_scenario(io_map),
        _set_variable_scenario(),
        _get_variable_scenario(),
    )
    return {scenario.name: scenario for scenario in scenarios}


#: The groups, in presentation order, with a line on what each family is for.
SCENARIO_GROUPS = (
    (
        "basic",
        "Basic control",
        "The RobotControl bits. Levels held until the robot answers, not events.",
    ),
    (
        "motion",
        "Motion",
        "The MotionControl bits and the move-id handshake every command shares.",
    ),
    (
        "data",
        "Data",
        "Reading what the fixed fields do not carry, and the robot's own variables.",
    ),
)


# ----------------------------------------------------------------------
# Command line
# ----------------------------------------------------------------------
def select_scenarios(
    scenarios: Dict[str, Scenario], name: str, on_robot: bool, allow_motion: bool
) -> Tuple[List[Scenario], List[Tuple[str, str]]]:
    """Choose which scenarios to run, and say why the others are left out.

    Args:
        scenarios: Every scenario, keyed by name.
        name: Value of ``--scenario``, or ``"all"``.
        on_robot: Whether the target is a real robot rather than the simulator.
        allow_motion: Whether moving the robot was allowed.

    Returns:
        A tuple ``(to run, [(name, reason skipped)])``.
    """
    wanted = list(scenarios.values()) if name == "all" else [scenarios[name]]
    runnable: List[Scenario] = []
    skipped: List[Tuple[str, str]] = []
    for scenario in wanted:
        if scenario.needs_simulator and on_robot:
            skipped.append(
                (scenario.name, "needs the simulator; a real robot will not fault on request")
            )
        elif scenario.needs_motion and on_robot and not allow_motion:
            skipped.append((scenario.name, "moves the robot; pass --allow-motion"))
        else:
            runnable.append(scenario)
    return runnable, skipped


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
        "--scenario",
        default="all",
        help="which scenario to run, or 'all' (default: %(default)s)",
    )
    parser.add_argument("--list-scenarios", action="store_true", help="list the scenarios and exit")
    parser.add_argument("--auto", action="store_true", help="do not wait for Enter between steps")
    parser.add_argument(
        "--step-delay",
        type=float,
        default=1.5,
        help="seconds to pause between steps when --auto is given (default: %(default)s)",
    )
    parser.add_argument(
        "--allow-motion",
        action="store_true",
        help="allow the scenarios that move a real robot; ignored against the simulator",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=DEFAULT_SETTLE_TIMEOUT_S,
        help="seconds allowed for the robot to answer a step (default: %(default)s)",
    )
    parser.add_argument(
        "--activation-time",
        type=float,
        default=0.5,
        help="simulated activation duration, long enough for Busy to be visible "
        "(default: %(default)s)",
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
    parser.add_argument("--no-color", action="store_true", help="plain output, no ANSI escapes")
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """Run one or more scenarios.

    Args:
        argv: Command line arguments, defaulting to ``sys.argv[1:]``.

    Returns:
        The process exit code.
    """
    args = build_parser().parse_args(argv)
    io_map = get_io_map()
    scenarios = available_scenarios(io_map)

    if args.list_scenarios:
        for group, title, blurb in SCENARIO_GROUPS:
            members = [s for s in scenarios.values() if s.group == group]
            if not members:
                continue
            print("\n{}\n  {}".format(title, blurb))
            for scenario in members:
                flags = []
                if scenario.requires:
                    flags.append("needs the robot {}".format(scenario.requires))
                if scenario.needs_motion:
                    flags.append("moves the robot")
                if scenario.needs_simulator:
                    flags.append("simulator only")
                print(
                    "  {:<14} {}{}".format(
                        scenario.name,
                        scenario.summary,
                        " [{}]".format(", ".join(flags)) if flags else "",
                    )
                )
        return 0

    if args.scenario != "all" and args.scenario not in scenarios:
        print(
            "unknown scenario {!r}; known: {}".format(args.scenario, ", ".join(scenarios)),
            file=sys.stderr,
        )
        return 1

    on_robot = args.address is not None
    runnable, skipped = select_scenarios(scenarios, args.scenario, on_robot, args.allow_motion)
    for name, reason in skipped:
        print("skipping {}: {}".format(name, reason))
    if skipped:
        print()
    if not runnable:
        print("nothing to run", file=sys.stderr)
        return 1

    transport, simulator, note = build_transport(
        io_map,
        address=args.address,
        udp_port=args.udp_port,
        activation_time_s=args.activation_time,
        homing_time_s=args.homing_time,
    )
    if note:
        print(note)
    print(
        "driving {} with I/O map version {}".format(describe_target(args.address), io_map.version)
    )
    print()

    status = 0
    try:
        transport.connect(args.address or "in-process")
        for scenario in runnable:
            walkthrough = Walkthrough(
                transport,
                io_map,
                color=not args.no_color,
                settle_timeout_s=args.settle_timeout,
            )
            walkthrough.simulator = simulator
            walkthrough.produce()
            status |= walkthrough.run(
                scenario, interactive=not args.auto, step_delay_s=args.step_delay
            )
            print()
    except FieldbusError as error:
        print("fieldbus error: {}".format(error), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # Release every request before letting go, whatever happened.
        try:
            transport.write_output_assembly(io_map.empty_output_assembly())
        except FieldbusError:
            pass
        transport.disconnect()
    return status


if __name__ == "__main__":
    sys.exit(main())
