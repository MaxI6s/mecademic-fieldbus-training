# mecademic-fieldbus-training

Training and demonstration tools for
[`mecademic-fieldbus`](../mecademicpy-cyclic), the standalone EtherNet/IP
scanner library for the Meca500.

Two tools, for the two halves of the same lesson:

| | |
| --- | --- |
| **`tag_dashboard`** | *What* the robot is publishing, live, in a browser: one tab for the live view — both process images as annotated hex, every tag decoded — and one tab per command, each walking its handshake step by step: set this bit, wait for that one. |
| **`command_walkthrough`** | *Why* it works: a narrated, pausable walk through what changes on the wire when a command is sent. |

Internal tool. The audience is a Mecademic application engineer showing an
integrator how the cyclic protocol behaves, so that they can reimplement the
same handshake on a PLC.

## Install

Two checkouts side by side:

```bash
git clone <this repo> && cd mecademic-fieldbus-training
pip install -e ../mecademicpy-cyclic
pip install -e .
```

Nothing else: the dashboard is `http.server` plus Server-Sent Events from the
standard library, and the library itself has no dependencies either (bar the
EtherNet/IP stack, which only the real-robot path needs).

## The walkthrough

```bash
python -m training.command_walkthrough --list-scenarios
python -m training.command_walkthrough --scenario send-command
```

With no `--address` it runs against an in-process simulator over
`LoopbackTransport`: no ports, no Forward Open, nothing to fail in front of an
audience. It pauses between steps; `--auto --step-delay 2` runs it unattended.

Each step prints what was sent, which **named tags** moved in the output
assembly, the hex region that carries them, what the robot answered in the input
assembly, and the one sentence to remember.

Thirteen sequences, one per command, in three groups. Each shows that command
and nothing else, so what you are looking at is always the response to the thing
you just sent.

**Basic control** — the `RobotControl` bits, every one a level held until the
robot answers:

| sequence | the mechanic |
| --- | --- |
| `activate` | a level-triggered request: held until `Activated` comes back, then released — and the robot stays activated |
| `deactivate` | its own bit, which wins over activate, and clears the homing reference |
| `home` | held until `RobotStatus_Homed`, which gates every motion command |
| `clear-fault` | there is no error bit — a non-zero `RobotStatus_Error` *is* the error |

**Motion** — the `MotionControl` bits and the handshake all 48 commands share:

| sequence | the mechanic |
| --- | --- |
| `pause` | stops the robot and keeps the queue |
| `resume` | the only bit in the assembly that acts on a **rising edge** |
| `clear-motion` | empties the queue; while it is held nothing can be queued |
| `send-command` | the move-id handshake, and why the arguments are staged first |
| `velocity` | the one they get wrong: a velocity command is **held** with a move id of `0`, never queued |
| `call-program` | an offline program started through that same handshake |

**Data** — what the 252 fixed bytes do not carry:

| sequence | the mechanic |
| --- | --- |
| `dynamic-data` | four slots: request a type, wait for the robot to echo it, then read six values |
| `set-variable` | identifiers 10000–19999 in `MotionCommand` write a robot variable |
| `get-variable` | the same identifier in a dynamic slot reads it back |

A sequence that assumes a state — most of them need the robot activated, the
motion ones need it homed — reaches it first and says so, rather than opening
with the same three activation steps every time:

```
preparation -- not part of the sequence
  robot activated and homed, so the sequence below starts from a known state
```

Against a real robot the sequences that move it refuse to start without
`--allow-motion`, and `clear-fault` is skipped because a robot will not fault on
cue:

```bash
python -m training.command_walkthrough --address 192.168.0.100 --scenario activate
```

Or against `python -m mock_robot` on sockets, which exercises the real stack --
Register Session, Forward Open, cyclic UDP frames -- rather than the in-process
shortcut:

```bash
python -m training.command_walkthrough --address 127.0.0.1 --scenario activate
```

## The dashboard

```bash
python -m training.tag_dashboard                 # simulator, read only
python -m training.tag_dashboard --allow-commands
python -m training.tag_dashboard --allow-motion --scenario-step-delay 2
```

Then open `http://127.0.0.1:8008/`.

### The tabs

A rail down the left side: **Live view** at the top, then the thirteen sequences
under their group. The live view holds everything that is continuously true —
both hex grids, the full tag table, the command buttons. Each sequence tab holds
one command.

### A sequence tab

Two pictures and almost no prose.

**The description** is one line and three tiles:

| tile | what it says | for `activate` |
| --- | --- | --- |
| **Trigger** | how the command is triggered, with the signal drawn as a glyph | ⎍ level request, released once confirmed |
| **Confirmed by** | the input tag that proves it took effect | `RobotStatus_Activated` |
| **Watch out** | the one trap | left set, the robot reactivates itself whenever it drops |

The glyph is the signal itself: a level held then released, a single rising
edge, a request and its echo, a value held high, a slot asked for and answered.
The full paragraph still exists — the terminal walkthrough prints it, because a
terminal is where reading is the point.

**The exchange** is two lanes, the scanner's and the robot's. Each sequence is
the same short chain — set the request, wait for the bit that confirms it,
reset the request — and the links draw it as a zigzag, unlabelled:

```
┌───────── OUTPUT ─────────┐            ┌┄┄┄┄┄┄┄┄┄ INPUT ┄┄┄┄┄┄┄┄┄┐
│ ① SET                    │            ┊ WAIT FOR                 ┊
│   RobotControl_Activate ─┼──────────▶ ┊   RobotStatus_Activated  ┊
│                          │          ╱ ┊                          ┊
│ ② RESET                ◀─┼─────────╱  ┊                          ┊
│   RobotControl_Activate  │            ┊                          ┊
└──────────────────────────┘            └┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┘
```

A link goes across from an action to the answer it waits for, then diagonally
from that answer down to the next action — which is what the answer unlocks.
Each step waits for the one tag that confirms it and nothing else: `Busy` and
the like are on the live view, not in the sequence. On the output side each chip
states the value its step writes — `RobotControl_Activate = True`, then `= False`
on the reset — with identifiers annotated (`MotionCommand = 1 · MoveJoints`) and
the move id shown as `previous + 1`, because its number depends on the run. Those
values are read back from a rehearsal of the scenario's own actions, so the page
cannot claim a value the step does not write. On the input side the chips are
live, and the box title states what they are waited for: `= True`, `≠ 0`.

The visual state does the explaining: what is still to come recedes, the link
of the step in progress turns blue and runs, an answered step turns green and
keeps **its own** `509 ms · 20 reads` — the number an integrator sizing a PLC
scan actually needs — and a step the robot never answered turns red.

Below that, a strip of just the bytes the sequence touches — `O4`, `O8`, `I8`,
`I14` — with their live hex value, flashing as they change.

### Every sequence puts its bits back down

Each `RobotControl` bit is a **request**: the vendor says the robot acts "as soon
(and as long as)" it is set. So a sequence holds a bit until the robot confirms
it, then releases it — and the robot stays where it was put. Releasing
`Activate` does not deactivate; only `Deactivate` does. Releasing `Pause` does
not resume; only a rising edge on `ResumeMotion` does. And `Setpoint` goes back
down once the robot has taken the command, because it says "this output carries
a command".

A bit left set keeps acting: a held `Activate` brings straight back up a robot
that a fault reset just powered down. So every sequence ends with its control
bits released and the next one starts from a clean output image — a rule the
test suite checks for all thirteen, by running each one and decoding what is
left. The one exception is `dynamic-data`: `DynamicTypeCfg0..3` are
configuration words, not control bits, and resetting them would blank the values
you just read.

While a sequence runs, the live view keeps up with it: the bytes its current
step touches are outlined in both hex grids and highlighted in the tag table, so
you can switch to the live tab mid-run and see the same handshake at full
width. "step tags only" in the tag filter narrows the table to just those. The
rail shows a pip against the sequence that is running, green when it finished
and red when the robot never answered.

The sequences are defined once, in `command_walkthrough.py`, and the page reads
them as data. The terminal tool and the dashboard cannot drift apart, which in a
training tool would mean teaching two different protocols.

Scenarios are gated exactly like the commands: `--allow-commands` enables the
ones that only change state, `--allow-motion` adds the ones that move the robot,
and `clear-fault` only appears against the simulator because a real robot will not
fault on request. Without either flag you can still *read* every sequence —
only the Run button is disabled.

### Style

The page follows the Mecademic Brand Guide v1.0 dark mapping, the same as
[`mecademic_demo_app`](https://github.com/maximeriera/mecademic_demo_app): the
palette is MIDNIGHT `#0C1A0F`, FOREST `#122616`, CLOUDS `#FCFDFC` and YUCATAN
`#5DEFBF`, corners are square, and the rail buttons are the uppercase tabs from
that app. Yellow, red, blue and orange are not brand colours; they stay because
they carry state meaning — blue for a step in progress, red for one with no
answer, amber for the output half of a handshake.

Archivo is **self-hosted** in `training/fonts/`, as the demo app self-hosts it,
so the page renders correctly on a cell network with no route out. Its OFL
licence travels with it. The server serves those two files and nothing else,
matched by exact path: a tool you point at a customer's robot is no place for a
static directory.

### The two process images

Each is a hex dump of one fixed-size frame, republished every cycle — the
fastest way to convey that this is not a stream of messages. Four things make
it readable rather than 252 numbers:

- **A legend**, one entry per tag group with its byte range: `RobotStatus 0–3`,
  `MotionStatus 4–15`, `JointSet 16–39`. Hover an entry and every byte outside
  that group dims, so the group is visible as a shape in the frame; click to
  pin it. This is how you answer "which bytes are the joint positions?" without
  counting.
- **A ruler** across the top, `+0 … +15`, with the columns in blocks of four.
  A byte number is read off the grid — row offset plus column — instead of
  counted out.
- **Five cycling tints**, with a dividing line where one group ends and the
  next begins. Tag groups are contiguous byte runs, so only *adjacent* groups
  ever need to differ: five muted shades do what twenty distinct colours could
  not, and they leave the one saturated blue free to mean *this byte just
  changed*.
- **Weight, not just colour**: a byte reading `00` steps back in grey, a byte
  carrying something is white and semibold. In a dim room with a projector
  that difference survives where a colour difference does not.

In the tag table the group prefix is dropped under its own heading — under
`RobotStatus` the rows read `Busy`, `Activated`, `Homed` — and the full name,
the one an integrator types into their configuration, is the first line of the
tooltip. A field that *is* its group, such as `MotionCommand`, keeps its whole
name. The filter still matches full names.

Every byte's tooltip names its group and the field that owns it — `byte 24 ·
JointSet · JointSet_Joint3Pos` — and the **bytes mapped to nothing** (8 in the
input image, 7 in the output) are drawn as dashed outlines rather than hidden,
because an integrator has to allocate them too.

Against a real robot:

```bash
python -m training.tag_dashboard --address 192.168.0.100   # read only
python -m training.tag_dashboard --address 127.0.0.1       # a mock_robot on sockets
```

A scanner listens on UDP 2222, which is also what `python -m mock_robot` binds
on the same machine. Pointing a tool at the loopback interface therefore picks a
free port instead and says so; `--udp-port 2222` overrides it, for a loopback
address that is really a tunnel to a robot.

### What "read only" means

A scanner has to produce something or the connection drops, so connecting
publishes an all-zero output image once. Read-only guarantees that nothing
changes it afterwards: the `/command` route is **not registered at all**, so
there is no endpoint and no button bar.

Enabling commands is deliberately graded:

| flag | what the page may do |
| --- | --- |
| *(none)* | nothing; read only |
| `--allow-commands` | activate, deactivate, reset, pause, resume, clear — none of which starts motion |
| `--allow-motion` | adds `Home` and **one** `MoveJoints`, whose target is fixed by `--demo-joint-target` on the command line. The page cannot invent a pose. |

The server binds the loopback interface by default and warns loudly if asked to
serve commands on anything else.

### Two things the page says out loud

It samples at 10 Hz while the connection runs at a 10 ms RPI, so it shows the
current process image, **not every frame on the wire**. And the greyed bytes are
unmapped, not broken. Both are misunderstandings a training session otherwise
installs for good.

## Running against a robot that a PLC is already driving

You cannot, yet. The robot grants one exclusive-owner connection and refuses a
second, so today the dashboard *is* the scanner. Watching alongside a customer's
PLC needs the monitoring-only connection the robot's EDS declares — tracked as
an open decision in the library's README.

## Tests

```bash
pytest -q
```

Everything runs against the in-process simulator on an ephemeral port: no robot,
no fixed ports, no sockets for the walkthrough at all. The suite covers the byte
map, the tag diffing, every sequence end to end, and the command gating —
including that `/command` really does return 404 when the dashboard is read only.

Two of them are worth knowing about, because they are what keeps the material
honest rather than merely working:

- every sequence is run start to finish against the simulator, and the list is
  taken from the sequences themselves, so a new one is covered the moment it is
  written;
- every step is run and its declared **we set** tags are compared against the
  output fields that actually moved. A step cannot quietly write a tag it does
  not show you — that check caught a real omission the first time it ran.

## Layout

```
training/
    assembly_view.py        annotate and diff images against the layout
    backend.py              decide what to connect to, and on which port
    command_walkthrough.py  the sequences: steps, tags, process descriptions
    tag_dashboard.py        the server and the sampler
    tag_dashboard.html      the page
    fonts/                  Archivo, self-hosted, with its OFL licence
tests/
```

`assembly_view.py` is the shared primitive, and `changed_fields()` is its heart:
it answers *which named tag moved*, not which byte. Every offset it uses comes
from `IoMap.describe_assembly()`, so these tools follow the specification
wherever the firmware takes it.
