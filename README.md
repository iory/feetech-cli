# feetech-cli

Command line tool to inspect and configure FEETECH STS/SMS/SCS serial bus
servos (STS3215 and friends) — change servo ids, bus speeds, modes and limits,
or watch a live status screen.

## Install

With [uv](https://docs.astral.sh/uv/), which installs the `feetech` command in its
own environment and on your path:

```console
$ uv tool install feetech-cli
```

or with pip:

```console
$ pip install feetech-cli
```

The package is also a Python library: `from feetech_cli import FeetechServoController`.

## Quick start

Plug in a USB servo adapter, give the servo its own power supply, then:

```console
$ feetech ports                    # which serial ports look like a servo bus
$ feetech scan                     # which servos answer (ids 1-16 by default)
$ feetech scan --all-baudrates     # ...when you do not know the bus speed
$ feetech                          # interactive status screen
```

The port and the bus speed are auto detected. Pass `--port` and `--baudrate`
to pin them.

No hardware handy? `--simulate` runs the whole interface against an in-process
servo bus (ids 1, 2 and 3), which is useful for trying the key bindings out:

```console
$ feetech --simulate
```

It only ever engages when you ask for it. A missing adapter or a silent bus
reports itself as such; it never quietly falls back to the simulator. The
screen header reads `--- SIMULATED SERVOS, NO HARDWARE ---` for as long as it
is running, so a simulation is never mistaken for a real bus.

## Changing a servo id

New STS3215 servos all leave the factory as **id 1 at 1 Mbps**. Every servo on
a bus needs its own id, so a robot with several servos starts with giving each
one a different id — **one servo at a time**.

### Walkthrough: numbering a batch of new servos

1. **Connect exactly one servo** to the USB adapter, and power the adapter
   from its own supply. USB alone does not power a servo.

2. Check that it answers, and at which id:

   ```console
   $ feetech scan
   Found 1 servo(s) on /dev/ttyUSB0 at 1.00Mbps:
     id   1  model   777  position  2048 (+0.0 deg)  12.1V  35C
   ```

3. Give it its new id. Numbering three servos 1, 2 and 3, the first one can
   keep id 1; the second becomes 2:

   ```console
   $ feetech set-id 1 2
   Change servo 1 to id 2? This writes the servo EEPROM. [y/N] y
   Servo 1 is now id 2.
   ```

   The id is stored in EEPROM, so it survives a power cycle.

4. **Label the servo** with its new id (a piece of masking tape is enough).
   Once they are mixed up, the only way to tell them apart again is to connect
   them one by one.

5. Unplug it, connect the next new servo, and repeat from step 2 with the
   next id (`feetech set-id 1 3` for the third).

6. With every servo numbered, daisy-chain them all and check the whole set
   answers:

   ```console
   $ feetech scan
   Found 3 servo(s) on /dev/ttyUSB0 at 1.00Mbps:
     id   1  model   777  position  2048 (+0.0 deg)  12.1V  35C
     id   2  model   777  position  2048 (+0.0 deg)  12.1V  35C
     id   3  model   777  position  2048 (+0.0 deg)  12.1V  35C
   ```

**Why one at a time.** Two new servos on the same bus both answer to id 1.
Their replies collide, and a write addressed to id 1 reaches both of them:
`set-id 1 2` would renumber both servos to 2, and the check that follows
would find "a servo at id 2" and report success. `set-id` cannot tell one
servo at an id from two, so keeping a single unnumbered servo on the bus is
up to you.

If `scan` finds nothing, the servo may have been set to another bus speed
before. `feetech scan --all-baudrates` tries every speed. Servos with an id
above 16 need `--max-id 253`.

### What set-id does

`set-id` refuses an id that another servo on the bus already answers to,
switches torque off first, unlocks the EEPROM, writes, locks it again, and
then pings the new id to confirm the change actually took. Add `-y` to skip
the prompt in scripts.

The same thing in the interactive screen: put the cursor on `Servo ID`, pick
the new id with the left/right arrows, press Enter. The chosen id is the one
shown in green.

| key | action |
|---|---|
| Up / Down | move between `Servo ID`, `Goal Position`, `Goal Velocity`, `Acceleration` |
| Left / Right | change **the selected row only** — see below |
| Enter | on `Servo ID`, write the highlighted id to the servo |
| `n` / `p` | switch to the next / previous servo on the bus |
| `r` | rescan the bus |
| `t` | toggle torque |
| `z` | move to 0 degrees |
| `0` | call the pose the servo is in **right now** 0 degrees (asks first, writes EEPROM) |
| `m` | cycle position / wheel / PWM / step mode |
| `q` | quit |

```
--- FEETECH Servo Status ---
Bus: /dev/ttyUSB0 @ 1.00Mbps   Servos: 1 4

>> Servo ID: 1  ->  new id: [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16]
   Goal Position: 2048 (+0.0 deg)
   Goal Velocity: 0
   Acceleration: 0

    Present Position: 2048 (+0.0 deg)
    Voltage: 12.1 V
    Temperature: 35 C
    Torque: Disabled
    Operating Mode: position (0)
    Baud Rate: 1.00Mbps
    EEPROM: locked
```

The arrows do something different on every row, so the screen names the
selected row and what they will do to it:

```
>> Servo ID: 1  ->  new id: [1 2 3 4 5 ...]
   Goal Position: 614 (-126.0 deg)
   ...
Now on Servo ID: Left/Right pick the id, Enter writes it to the servo
```

The cursor starts on `Servo ID`, where the arrows pick an id rather than move
the horn. Press Down to reach `Goal Position` before expecting the servo to
move.

## Swapping servos while it runs

The interactive screen does not need restarting when you change servos. When
the bus goes quiet it switches to a waiting screen and keeps looking:

```
--- FEETECH Servo Status ---

No servo is answering.

Plug a servo in and it will be picked up automatically.
Retrying once a second. Also worth checking:
  - the USB adapter is still connected
  - the servo has its own power supply, USB alone is not enough
  - the servo id is between 1 and 16 (widen with --min-id / --max-id)

'r' retry now, 'q' quit
```

The id range is rescanned each time, so the replacement is found even when it
answers to a different id. Unplugging the USB adapter itself is handled too:
the port is closed and reopened rather than just rescanned.

It also starts this way. `feetech` with nothing on the bus opens the waiting
screen instead of exiting, so you can start the tool first and connect a servo
afterwards. The one shot commands (`scan`, `info`, `set-id`, ...) still fail
straight away, since they have nothing to wait for.

## Torque

**A position command energises the servo by itself.** Measured on an STS3215:
with `torque_enable` explicitly set to 0, writing `goal_position` took it back
to 1 and the horn moved. Writes to `lock`, `homing_offset` and `acceleration`
did not do this, so it is specific to the position command.

So the arrow keys and `feetech move` always move the servo, even from a limp
standstill, and the joint stops being back drivable at that moment:

```
>> Goal Position: 2216 (+14.8 deg)  (arrows will energise the servo)
   Present Position: 2216 (+14.8 deg)
   Torque: Enabled
```

To make a joint free to move by hand, switch torque **off** and leave it off:
press `t`, or run `feetech torque 1 off`. While it is off, `Present Position`
tracks the joint as you move it. Send any position command and it stiffens
again.

Pressing `t` to energise holds the position the horn is actually in, not the
goal last commanded. Otherwise the servo would snap to a stale goal the moment
it came on, which on an assembled robot can break something.

## When a servo will not move

`selftest` is the check to run first. It verifies the settings that keep a
servo still, then commands a short series of moves with torque on and reads
back where the horn actually ended up, so "the tool is not sending anything"
is separated from "the servo is not acting on it":

```console
$ feetech selftest 1
Settings that would stop it moving:
  [ok  ] operating mode is position: 0 (position)
  [ok  ] torque limit above zero: 1000
  ...
Moves, starting from 996:
   target   before  reached   error   secs  result
     1046      996     1044      -2   0.44  moved and reached the target
     ...
PASS: 5 of 5 commanded moves reached their target.
```

Moves stay within `--span` counts of where the servo started (100 by default,
about 9 degrees) and the servo is put back where it was found, with torque
restored to whatever it was.

## Zeroing a joint

Two different things, both available:

**Move the joint to 0 degrees** — the servo travels to encoder count 2048:

```console
$ feetech move 1 --degree 0
```

or press `z` in the interactive screen.

**Call the joint's current pose 0 degrees** — nothing moves, the servo just
starts reporting the pose it is already in as 0. This is how a joint is zeroed
after assembly:

```console
$ feetech set-zero 1
Servo 1 currently reports 311 (-152.7 deg).
Call this pose 2048 (+0.0 deg)? This writes homing_offset to the servo EEPROM.
The servo will not move. [y/N] y
Servo 1 now reports 2048 (+0.0 deg) in the same pose.
homing_offset = -1652
```

It writes the `homing_offset` register, where
`present_position = (raw_encoder - homing_offset) mod 4096` (verified on an
STS3215). Torque is left as it was found, and the goal is re-pointed at the
new reading so re-energising does not make the joint jump.

The wrap matters the second time a joint is zeroed. A servo that already
carries an offset reports a position that has crossed the 0/4095 boundary —
raw 628 with an offset of 904 reads 3820, not -276 — so the reading is
brought back into range before the new offset is worked out. Skipping that
asks the servo for an offset the register cannot hold and refuses a zeroing
that would have fitted.

In the interactive screen this is the `0` key. It asks before writing:

```
Call this pose (2134, +7.6 deg) 0 degrees? This writes homing_offset to the
servo EEPROM. The servo will not move. Press 'y' to confirm, any other key
to cancel.
```

To name the current pose something other than zero:

```console
$ feetech set-zero 1 --degree 90
$ feetech set-zero 1 --position 1024
```

The offset register holds ±2047 counts, so the pose has to be within about
180 degrees of the target; `set-zero` refuses rather than writing a wrapped
value.

## Other commands

```console
$ feetech info 1                   # dump the control table
$ feetech info 1 --all             # ...every register, not just the useful ones
$ feetech set-baud 1 115200        # change the bus speed and reconnect
$ feetech move 1 --degree 90       # command a position
$ feetech move 1 --position 3072   # ...in encoder counts
$ feetech torque 1 off             # let the horn move freely
$ feetech mode 1 wheel             # continuous rotation
$ feetech read 1 present_load
$ feetech write 1 acceleration 20
$ feetech registers                # names and addresses of the control table
$ feetech factory-reset 1
```

## More than one servo on the bus

Everything acts on one servo at a time. `scan` lists them all; every other
command works on the id it is given.

```console
$ feetech scan
Found 3 servo(s) on /dev/ttyUSB0 at 1.00Mbps:
  id   1  model   777  position  2048 (+0.0 deg)  12.1V  35C
  id   2  model   777  position  1024 (-90.0 deg)  12.1V  36C
  id   3  model   777  position  3072 (+90.0 deg)  12.0V  35C
```

`info`, `set-zero` and `selftest` allow the id to be left out, but only when
exactly one servo answers. With several on the bus they name them and stop
rather than picking one:

```console
$ feetech info
several servos answered (1, 2, 3), name the one to use
```

The rest — `move`, `torque`, `mode`, `read`, `write`, `set-baud`,
`factory-reset` — always want the id. `set-id` additionally refuses to create
a collision:

```console
$ feetech set-id 1 2
id 2 is already used by another servo on this bus
```

There is no `--all` flag and no broadcast command. An EEPROM write that lands
on every joint of an assembled robot is not something a mistyped command
should be able to do. `FeetechServoController.sync_write_positions()` does
exist, and commands several servos in one packet, but it is a Python API
rather than a subcommand.

The interactive screen lists every servo found, with the selected one in
cyan:

```
Bus: /dev/ttyUSB0 @ 1.00Mbps   Servos: 1 2 3
```

`n` and `p` move to the next and previous servo, `r` rescans. It opens on the
lowest id. Only the selected servo is read each frame, so the refresh rate
does not drop as servos are added to the bus, and every key — the arrows,
`t`, `z`, `m`, `0` — applies to that servo alone. A rescan keeps you on the
servo you were watching when it is still there, and falls back to the lowest
id when it is not.

## Servo id range

Ids 1-16 are scanned by default, which covers a typical arm or hand. FEETECH
servos accept ids up to 253; widen the range when you need to:

```console
$ feetech --max-id 253 scan
```

`--min-id` and `--max-id` also set the ids offered by the interactive screen.

## SCS series

The older SCS servos store 16 bit registers big endian. Pass `--scs` to talk
to them.

## Safety notes

- Writing the EEPROM (`set-id`, `set-baud`, `mode`, `factory-reset`) asks for
  confirmation unless `-y` is given, and refuses to run unattended without it.
- `set-id` and `set-baud` verify the change by re-addressing the servo
  afterwards; they raise instead of reporting success if the servo does not
  answer at its new identity.
- If a servo goes missing after a speed change, find it again with
  `feetech scan --all-baudrates`.

## Development

```console
$ uv sync --all-groups
$ uv run pytest
$ uv run ruff check .
```

The test suite runs against the simulated servo bus in
`feetech_cli/simulator.py`, so it needs no hardware. It covers the packet
layer, the EEPROM lock dance, id and baud rate changes, and the CLI.

`tests/test_terminal.py` additionally drives the interactive screen through a
real pty, sending real arrow key and Enter sequences, which is the only way to
cover `readchar` and the escape handling for real.

## Protocol reference

FEETECH servos speak a Dynamixel-1.0 style protocol:

```
instruction : 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
status      : 0xFF 0xFF ID LENGTH ERROR       PARAM... CHECKSUM
```

`LENGTH` is `len(params) + 2` and `CHECKSUM` is
`~(ID + LENGTH + INSTRUCTION + sum(params)) & 0xFF`.

Baud rate register values:

| value | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| bps | 1000000 | 500000 | 250000 | 128000 | 115200 | 76800 | 57600 | 38400 |

Note that [LeRobot's table][lerobot] lists values 5-7 as 57600/38400/19200
instead. The table above is the one used here, and matches three independent
implementations.

Sources:

- STS3215 register reference: https://github.com/commanderfun/STS3215/blob/main/REGISTER_REFERENCE.md
- Hello Robot Stretch driver (baud table): https://github.com/hello-robot/stretch4_body/blob/master/stretch4_body/core/feetech/feetech_SM_servo.py
- AkariGroup feetech_setup (baud table): https://github.com/AkariGroup/feetech_setup/blob/main/set_baudrate.py
- LeRobot control table and sign-magnitude encodings: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py
- FEETECH start tutorial: https://www.feetechrc.com/Data/feetechrc/upload/file/20201127/start%20%20tutorial201015.pdf
- Factory default id 1: https://www.waveshare.com/wiki/ST3215_Servo ; id 1 and 1 Mbps: the STS3215 register reference above
- LeRobot SO-101 assembly, numbering the motors one at a time: https://huggingface.co/docs/lerobot/main/en/assemble_so101

[lerobot]: https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py

## License

MIT
