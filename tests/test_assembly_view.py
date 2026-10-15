"""Tests of the annotation and diffing primitive.

Byte fixtures only: no robot, no sockets, no timing.
"""

import pytest
from mecademic_fieldbus.io_map import INPUT_ASSEMBLY, OUTPUT_ASSEMBLY, IoMap
from mecademic_fieldbus.robot_classes import MotionCommand, MotionControl, RobotStatus

from training.assembly_view import (
    AssemblyView,
    format_diff,
    format_field_table,
    format_hex_dump,
    format_value,
    group_of,
)


def test_every_byte_is_owned_or_reported_unmapped(io_map: IoMap) -> None:
    """The byte map accounts for every byte of both assemblies exactly once.

    This is the invariant the hex grid rests on: a byte that belonged to
    nothing *and* was not reported as unmapped would render as a silent hole.
    """
    for assembly in (INPUT_ASSEMBLY, OUTPUT_ASSEMBLY):
        view = AssemblyView(io_map, assembly)
        owners = view.byte_owners()
        assert len(owners) == view.description.size_bytes
        unmapped = set(view.unmapped_bytes())
        for offset, names in enumerate(owners):
            assert bool(names) != (offset in unmapped), offset


def test_the_unmapped_bytes_are_the_ones_the_vendor_file_leaves_out(io_map: IoMap) -> None:
    """The counts are worth pinning: they are a teaching point, not an accident."""
    assert len(AssemblyView(io_map, INPUT_ASSEMBLY).unmapped_bytes()) == 8
    assert len(AssemblyView(io_map, OUTPUT_ASSEMBLY).unmapped_bytes()) == 7


def test_bytes_shared_by_several_bits_list_every_owner(io_map: IoMap) -> None:
    """A byte packed with bit fields names all of them, not just the first."""
    view = AssemblyView(io_map, OUTPUT_ASSEMBLY)
    owners = view.byte_owners()[0]
    assert "RobotControl_Activate" in owners
    assert "RobotControl_Deactivate" in owners
    assert len(owners) > 2


def test_changed_fields_reports_the_move_id_handshake(io_map: IoMap) -> None:
    """Staging a command and triggering it moves exactly the expected tags."""
    view = AssemblyView(io_map, OUTPUT_ASSEMBLY)
    before = io_map.empty_output_assembly()
    staged = io_map.encode_motion_command(
        MotionCommand.build(io_map.motion_command_id("MoveJoints"), (0.0,) * 5 + (10.0,)), before
    )
    triggered = io_map.encode_motion_control(MotionControl(move_id=1, setpoint=True), staged)

    staging = {change.field.name for change in view.changed_fields(before, staged)}
    assert staging == {"MotionCommand", "MotionCommand_Arg6"}

    handshake = {change.field.name for change in view.changed_fields(staged, triggered)}
    assert handshake == {"MotionControl_MoveId", "MotionControl_Setpoint"}


def test_changed_fields_can_ignore_the_free_running_counters(io_map: IoMap) -> None:
    """Filtering the counters leaves the change that matters, and only it."""
    from mecademic_fieldbus.robot_classes import RobotTimestamp

    view = AssemblyView(io_map, INPUT_ASSEMBLY)
    before = io_map.encode_timestamp(RobotTimestamp(seconds=1, dynamic_data_cycles=1))
    after = io_map.encode_timestamp(
        RobotTimestamp(seconds=2, dynamic_data_cycles=2),
        io_map.encode_status(RobotStatus(activated=True), before),
    )
    assert len(view.changed_fields(before, after)) == 3
    filtered = view.changed_fields(before, after, ignore=("Timestamp_Second", "DynamicDataCycles"))
    assert [change.field.name for change in filtered] == ["RobotStatus_Activated"]


def test_changed_bytes_keeps_a_byte_shared_with_something_meaningful(io_map: IoMap) -> None:
    """A byte is only hidden when every field on it is ignored."""
    view = AssemblyView(io_map, INPUT_ASSEMBLY)
    before = io_map.empty_input_assembly()
    after = io_map.encode_status(RobotStatus(activated=True), before)
    assert view.changed_bytes(before, after, ignore=("RobotStatus_Busy",)) == (0,)
    assert view.changed_bytes(before, after, ignore=("RobotStatus_Activated",)) == (0,)


def test_an_image_of_the_wrong_size_is_refused(io_map: IoMap) -> None:
    """A truncated image fails loudly rather than rendering nonsense."""
    view = AssemblyView(io_map, INPUT_ASSEMBLY)
    with pytest.raises(ValueError):
        view.bytes_view(bytes(4))


def test_group_of_splits_on_the_vendor_naming(io_map: IoMap) -> None:
    """Groups come from the vendor's own ``Group_Detail`` naming."""
    view = AssemblyView(io_map, INPUT_ASSEMBLY)
    assert group_of(view.field("RobotStatus_Busy")) == "RobotStatus"
    assert group_of(view.field("JointSet_Joint1Pos")) == "JointSet"


def test_format_value_appends_the_unit(io_map: IoMap) -> None:
    """A value is rendered with the unit the specification carries."""
    view = AssemblyView(io_map, INPUT_ASSEMBLY)
    assert format_value(view.field("JointSet_Joint1Pos"), 10.5) == "10.5 deg"
    assert format_value(view.field("RobotStatus_Busy"), True) == "true"
    assert format_value(view.field("RobotStatus_Error"), 0) == "0"


def test_format_diff_says_so_when_nothing_moved(io_map: IoMap) -> None:
    """An unchanged image is reported as such, not as an empty block."""
    view = AssemblyView(io_map, OUTPUT_ASSEMBLY)
    raw = io_map.empty_output_assembly()
    assert "no change" in format_diff(view, raw, raw, color=False)


def test_format_diff_names_the_tags_before_showing_bytes(io_map: IoMap) -> None:
    """The named change comes first; the hex is supporting evidence."""
    view = AssemblyView(io_map, OUTPUT_ASSEMBLY)
    before = io_map.empty_output_assembly()
    after = io_map.encode_motion_control(MotionControl(move_id=7), before)
    rendered = format_diff(view, before, after, color=False)
    assert "MotionControl_MoveId: 0 -> 7" in rendered
    assert rendered.index("MotionControl_MoveId") < rendered.index("RobotControl")


def test_the_renderers_cover_the_whole_image(io_map: IoMap) -> None:
    """The hex dump and the field table render every byte and every field."""
    view = AssemblyView(io_map, INPUT_ASSEMBLY)
    raw = io_map.empty_input_assembly()
    dump = format_hex_dump(view, raw, color=False)
    assert len(dump.splitlines()) == (view.description.size_bytes + 15) // 16
    table = format_field_table(view, raw, color=False)
    for field in view.fields:
        assert field.name in table
