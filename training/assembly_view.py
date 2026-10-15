"""Annotate and diff raw assembly images against the layout that describes them.

This is the primitive both tools are built on.  It answers the three questions a
trainee actually asks when looking at 252 bytes of hex:

* **which tag does this byte belong to?** -- :meth:`AssemblyView.byte_owners`,
  including the bytes that belong to nothing, which the vendor file leaves
  unmapped and which are worth showing rather than hiding;
* **what just changed?** -- :meth:`AssemblyView.changed_fields`, by *named tag*
  rather than by byte, which is the whole point;
* **what does that value mean?** -- :func:`format_value`, using the unit and
  description the specification carries.

Every offset comes from :meth:`~mecademic_fieldbus.io_map.base.IoMap.describe_assembly`.
Nothing here hard-codes one, which is what keeps these tools honest when the
robot's firmware layout moves.

Pure and side-effect free: no sockets, no robot, no printing.  That is what
makes it testable on byte fixtures alone.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from mecademic_fieldbus.io_map import INPUT_ASSEMBLY, AssemblyDescription, IoMap
from mecademic_fieldbus.io_map.codec import FieldSpec, FieldValue

__all__ = [
    "AssemblyView",
    "ByteView",
    "FieldChange",
    "format_diff",
    "format_field_table",
    "format_hex_dump",
    "format_value",
    "group_of",
    "UNMAPPED_GROUP",
]

#: Group reported for a byte that belongs to no field, or to fields of several
#: groups at once.
UNMAPPED_GROUP = ""

#: ANSI escapes, used only when a caller asks for colour.
_BOLD = "\033[1m"
_REVERSE = "\033[7m"
_DIM = "\033[2m"
_RESET = "\033[0m"


def group_of(field: FieldSpec) -> str:
    """Return the group a field belongs to, from its name.

    The vendor names every field ``Group_Detail`` -- ``RobotStatus_Activated``,
    ``MotionControl_Setpoint``, ``JointSet_Joint1Pos`` -- so the prefix is a
    free and accurate way to colour a display and to order a table.  It is a
    naming convention rather than a property of the layout, which is why it is
    derived here and not in the library.

    Args:
        field: Field to classify.

    Returns:
        The prefix, or the whole name when it carries no underscore.
    """
    return field.name.split("_", 1)[0]


def format_value(field: FieldSpec, value: FieldValue) -> str:
    """Render one decoded value the way a human reads it.

    Args:
        field: Field the value was decoded from.
        value: Decoded value.

    Returns:
        A short string, with the field's unit appended when it has one.
    """
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, tuple):
        return ", ".join(format_value(field, element) for element in value)
    if isinstance(value, float):
        rendered = "{:.3f}".format(value).rstrip("0").rstrip(".")
        rendered = rendered or "0"
    else:
        rendered = str(value)
    return "{} {}".format(rendered, field.unit) if field.unit else rendered


@dataclass(frozen=True)
class ByteView:
    """One byte of an assembly, with what it belongs to.

    Attributes:
        offset: Position of the byte in the assembly.
        value: Current value.
        previous: Value in the image this one was compared against, or ``None``
            when there was nothing to compare against.
        owners: Names of every field that occupies part of this byte, in
            declaration order.  Empty for a byte the vendor file leaves
            unmapped.
        group: Group shared by every owner, or :data:`UNMAPPED_GROUP` when the
            byte is unmapped or shared across groups.
    """

    offset: int
    value: int
    previous: Optional[int]
    owners: Tuple[str, ...]
    group: str

    @property
    def changed(self) -> bool:
        """Whether this byte differs from the image it was compared against."""
        return self.previous is not None and self.previous != self.value

    @property
    def mapped(self) -> bool:
        """Whether any field claims part of this byte."""
        return bool(self.owners)


@dataclass(frozen=True)
class FieldChange:
    """One named tag whose value moved between two images.

    Attributes:
        field: The field that changed.
        before: Value in the earlier image.
        after: Value in the later image.
    """

    field: FieldSpec
    before: FieldValue
    after: FieldValue

    @property
    def position(self) -> str:
        """The field's position, as ``byte N`` or ``byte N.bit M``."""
        if self.field.is_bool:
            return "byte {}.bit {}".format(self.field.byte_offset, self.field.bit_offset)
        return "byte {}".format(self.field.byte_offset)

    def __str__(self) -> str:
        """Render the change as ``name: before -> after (position)``."""
        return "{}: {} -> {}  ({})".format(
            self.field.name,
            format_value(self.field, self.before),
            format_value(self.field, self.after),
            self.position,
        )


class AssemblyView:
    """A layout-aware view over the raw images of one assembly.

    Args:
        io_map: Map describing the assembly.
        assembly: Which assembly this view annotates.

    Raises:
        FieldbusIoMapError: If there is no assembly by that name.
    """

    def __init__(self, io_map: IoMap, assembly: str = INPUT_ASSEMBLY) -> None:
        self._io_map = io_map
        self._assembly = assembly
        self._description = io_map.describe_assembly(assembly)
        self._owners = self._compute_owners()

    @property
    def description(self) -> AssemblyDescription:
        """The assembly's geometry and fields, as the library reports them."""
        return self._description

    @property
    def assembly(self) -> str:
        """Name of the assembly this view annotates."""
        return self._assembly

    @property
    def fields(self) -> Tuple[FieldSpec, ...]:
        """Every field of the assembly, in declaration order."""
        return self._description.fields

    def field(self, name: str) -> FieldSpec:
        """Return one field by name.

        Args:
            name: Field name.

        Returns:
            The matching field.

        Raises:
            KeyError: If the assembly has no such field.
        """
        for field in self._description.fields:
            if field.name == name:
                return field
        raise KeyError(name)

    def byte_owners(self) -> Tuple[Tuple[str, ...], ...]:
        """Return, for every byte of the assembly, the fields that occupy it.

        Returns:
            One tuple of field names per byte, indexed by offset.  An empty
            tuple means the vendor file maps nothing there.
        """
        return self._owners

    def unmapped_bytes(self) -> Tuple[int, ...]:
        """Return the offsets the vendor file maps no field to.

        Worth showing rather than hiding: a trainee who sees them greyed out
        learns that the assembly is a fixed-size frame with room reserved in
        it, which is exactly what they will have to allocate on their own side.

        Returns:
            The unmapped offsets, ascending.
        """
        return tuple(offset for offset, owners in enumerate(self._owners) if not owners)

    def bytes_view(self, raw: bytes, previous: Optional[bytes] = None) -> Tuple[ByteView, ...]:
        """Annotate every byte of an image, optionally against an earlier one.

        Args:
            raw: Image to annotate.
            previous: Image to compare against, or ``None``.

        Returns:
            One :class:`ByteView` per byte of the assembly.

        Raises:
            ValueError: If an image is shorter than the assembly.
        """
        self._check_length(raw)
        if previous is not None:
            self._check_length(previous)
        return tuple(
            ByteView(
                offset=offset,
                value=raw[offset],
                previous=None if previous is None else previous[offset],
                owners=self._owners[offset],
                group=self._group_at(offset),
            )
            for offset in range(self._description.size_bytes)
        )

    def changed_bytes(
        self, before: bytes, after: bytes, ignore: Sequence[str] = ()
    ) -> Tuple[int, ...]:
        """Return the offsets that differ between two images.

        Args:
            before: Earlier image.
            after: Later image.
            ignore: Field names to disregard.  A byte is dropped only when
                *every* field occupying it is ignored, so a counter sharing a
                byte with something meaningful still shows.

        Returns:
            The differing offsets, ascending.

        Raises:
            ValueError: If an image is shorter than the assembly.
        """
        self._check_length(before)
        self._check_length(after)
        hidden = set(ignore)
        return tuple(
            offset
            for offset in range(self._description.size_bytes)
            if before[offset] != after[offset]
            and not (self._owners[offset] and set(self._owners[offset]) <= hidden)
        )

    def changed_fields(
        self, before: bytes, after: bytes, ignore: Sequence[str] = ()
    ) -> Tuple[FieldChange, ...]:
        """Return the named tags whose value moved between two images.

        This is the question worth answering in a demonstration: not *which
        byte* changed, but *which tag*.  A trainee who can read "the move id
        went from 0 to 1" has learned the protocol; one looking at ``byte 4``
        has not.

        Args:
            before: Earlier image.
            after: Later image.
            ignore: Field names to leave out, for the free-running counters a
                live robot changes on every single frame.

        Returns:
            One change per field whose decoded value differs, in declaration
            order.

        Raises:
            ValueError: If an image is shorter than the assembly.
        """
        self._check_length(before)
        self._check_length(after)
        hidden = set(ignore)
        old = self._io_map.decode_all(self._assembly, before)
        new = self._io_map.decode_all(self._assembly, after)
        return tuple(
            FieldChange(field=field, before=old[field.name], after=new[field.name])
            for field in self._description.fields
            if field.name not in hidden and old[field.name] != new[field.name]
        )

    def decode(self, raw: bytes) -> Dict[str, FieldValue]:
        """Decode every field of an image.

        Args:
            raw: Image to decode.

        Returns:
            A mapping of field name to decoded value.
        """
        return self._io_map.decode_all(self._assembly, raw)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _compute_owners(self) -> Tuple[Tuple[str, ...], ...]:
        """Build the byte-to-fields map from the field offsets.

        Returns:
            One tuple of field names per byte of the assembly.
        """
        owners: List[List[str]] = [[] for _ in range(self._description.size_bytes)]
        for field in self._description.fields:
            for offset in range(field.byte_offset, field.end_byte_offset):
                if 0 <= offset < len(owners):
                    owners[offset].append(field.name)
        return tuple(tuple(names) for names in owners)

    def _group_at(self, offset: int) -> str:
        """Return the group shared by the owners of one byte.

        Args:
            offset: Byte offset.

        Returns:
            The common group, or :data:`UNMAPPED_GROUP` when unmapped or mixed.
        """
        groups = {name.split("_", 1)[0] for name in self._owners[offset]}
        return groups.pop() if len(groups) == 1 else UNMAPPED_GROUP

    def _check_length(self, raw: bytes) -> None:
        """Verify an image is large enough for this assembly.

        Args:
            raw: Image to check.

        Raises:
            ValueError: If it is shorter than the assembly.
        """
        if len(raw) < self._description.size_bytes:
            raise ValueError(
                "{} assembly needs {} bytes, got {}".format(
                    self._assembly, self._description.size_bytes, len(raw)
                )
            )


def format_hex_dump(
    view: AssemblyView,
    raw: bytes,
    previous: Optional[bytes] = None,
    columns: int = 16,
    color: bool = True,
    only_offsets: Optional[Sequence[int]] = None,
) -> str:
    """Render an image as an annotated hex dump.

    Args:
        view: View describing the assembly.
        raw: Image to render.
        previous: Image to compare against, so changed bytes stand out.
        columns: Bytes per line.
        color: Whether to highlight with ANSI escapes.
        only_offsets: Render only the lines containing these offsets, which is
            how a diff shows just the region that moved.

    Returns:
        The dump, one line per ``columns`` bytes, with the groups present on
        each line named in the margin.
    """
    views = view.bytes_view(raw, previous)
    wanted = None if only_offsets is None else {offset // columns for offset in only_offsets}
    lines: List[str] = []
    for start in range(0, len(views), columns):
        if wanted is not None and start // columns not in wanted:
            continue
        chunk = views[start : start + columns]
        cells = []
        for byte in chunk:
            cell = "{:02x}".format(byte.value)
            if color and byte.changed:
                cell = _REVERSE + cell + _RESET
            elif color and not byte.mapped:
                cell = _DIM + cell + _RESET
            cells.append(cell)
        groups = []
        for byte in chunk:
            if byte.group and byte.group not in groups:
                groups.append(byte.group)
        lines.append(
            "{:>4}  {}  {}".format(start, " ".join(cells), ", ".join(groups) or "unmapped")
        )
    return "\n".join(lines)


def format_field_table(
    view: AssemblyView,
    raw: bytes,
    names: Optional[Sequence[str]] = None,
    color: bool = True,
) -> str:
    """Render the decoded fields of an image as a table.

    Args:
        view: View describing the assembly.
        raw: Image to decode.
        names: Only these fields, in this order.  Defaults to all of them.
        color: Whether to emphasise group headings with ANSI escapes.

    Returns:
        A table of name, position, type and decoded value, grouped by prefix.
    """
    values = view.decode(raw)
    fields = view.fields if names is None else tuple(view.field(name) for name in names)
    lines: List[str] = []
    current_group = None
    for field in fields:
        group = group_of(field)
        if group != current_group:
            current_group = group
            heading = _BOLD + group + _RESET if color else group
            lines.append(heading)
        position = (
            "byte {}.{}".format(field.byte_offset, field.bit_offset)
            if field.is_bool
            else "byte {}".format(field.byte_offset)
        )
        lines.append(
            "  {:<38} {:<12} {:<9} {}".format(
                field.name, position, field.type, format_value(field, values[field.name])
            )
        )
    return "\n".join(lines)


def format_diff(
    view: AssemblyView,
    before: bytes,
    after: bytes,
    context_bytes: int = 2,
    color: bool = True,
    ignore: Sequence[str] = (),
) -> str:
    """Render what changed between two images, by tag and then by byte.

    Args:
        view: View describing the assembly.
        before: Earlier image.
        after: Later image.
        context_bytes: How many bytes either side of a change to keep when
            selecting the hex lines to show.
        color: Whether to highlight with ANSI escapes.
        ignore: Field names to leave out of the listing.  Their number is
            still reported, because hiding them silently would teach that a
            live robot's image is static, which it is not.

    Returns:
        The named changes followed by the hex lines that carry them, or a short
        note when nothing moved.
    """
    changes = view.changed_fields(before, after, ignore=ignore)
    changed = view.changed_bytes(before, after, ignore=ignore)
    suppressed = len(view.changed_fields(before, after)) - len(changes)
    if not changes and not changed:
        note = "  (no change)"
        if suppressed:
            note += ", apart from {} free-running counter(s)".format(suppressed)
        return note

    lines: List[str] = []
    for change in changes:
        lines.append("  {}".format(change))
    if not changes and changed:
        lines.append("  (bytes changed in a region no field is mapped to)")
    if suppressed:
        lines.append(
            "  (plus {} free-running counter(s), which change every frame)".format(suppressed)
        )
    if changed:
        window = sorted(
            {
                offset
                for position in changed
                for offset in range(position - context_bytes, position + context_bytes + 1)
                if 0 <= offset < view.description.size_bytes
            }
        )
        lines.append("")
        lines.append(format_hex_dump(view, after, before, color=color, only_offsets=window))
    return "\n".join(lines)
