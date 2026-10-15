"""Training and demonstration tools for the cyclic fieldbus library.

Two tools, one shared primitive:

* :mod:`training.tag_dashboard` -- a live view of both process images in a
  browser, decoded and raw, for showing what the robot is publishing right now.
* :mod:`training.command_walkthrough` -- a narrated, pausable walk through what
  changes on the wire when a command is sent, for showing *why* it works.
* :mod:`training.assembly_view` -- the annotation and diffing both are built on.

Everything reads its offsets from the library's I/O map, never from a constant,
so these tools cannot drift from the specification the robot is actually
running.
"""

__version__ = "0.1.0"
