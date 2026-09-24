"""Interactive terminal UI for inspecting and reconfiguring servos."""

import queue
import sys
import threading
import time

import readchar
from colorama import Fore
from colorama import Style

from feetech_cli.protocol import FeetechError
from feetech_cli.protocol import FeetechTimeoutError
from feetech_cli.registers import BAUDRATE_TABLE
from feetech_cli.registers import CENTER_POSITION
from feetech_cli.registers import OPERATING_MODES
from feetech_cli.registers import RESOLUTION
from feetech_cli.registers import format_baud
from feetech_cli.registers import position_to_degree

#: Encoder counts moved per arrow key press, about five degrees.
POSITION_STEP = RESOLUTION // 72

#: Velocity change per arrow key press, in encoder counts per second.
VELOCITY_STEP = 100

#: Acceleration change per arrow key press.
ACCELERATION_STEP = 5

#: Rows the user can move the cursor onto.
SELECTABLE_ROWS = ["Servo ID", "Goal Position", "Goal Velocity", "Acceleration"]

#: What the left and right arrows do on each selectable row. Shown for the
#: selected row: the arrows mean something completely different depending on
#: where the cursor is, and a generic "Left/Right change it" hides that.
ROW_HINTS = {
    "Servo ID": "Left/Right pick the id, Enter writes it to the servo",
    "Goal Position": "Left/Right move the servo by about 5 degrees",
    "Goal Velocity": "Left/Right change the target velocity",
    "Acceleration": "Left/Right change the acceleration ramp",
}

#: Read only rows, in display order.
STATUS_ROWS = [
    "Present Position",
    "Present Velocity",
    "Present Load",
    "Voltage",
    "Temperature",
    "Current",
    "Torque",
    "Operating Mode",
    "Moving",
    "Position Limits",
    "Homing Offset",
    "Baud Rate",
    "EEPROM",
]

#: Registers fetched for one frame. Each one costs a bus round trip, which is
#: why the screen refreshes on a timer rather than as fast as it can.
STATUS_REGISTERS = [
    "goal_position",
    "goal_velocity",
    "acceleration",
    "present_position",
    "present_velocity",
    "present_load",
    "present_voltage",
    "present_temperature",
    "present_current",
    "torque_enable",
    "operating_mode",
    "moving",
    "min_position_limit",
    "max_position_limit",
    "homing_offset",
    "baud_rate",
    "lock",
]

#: Seconds between two reads of the servo state.
REFRESH_INTERVAL = 0.25

#: Seconds the main loop sleeps while waiting for a key press.
POLL_INTERVAL = 0.02

#: Seconds between attempts to find a servo again after the bus went quiet.
RECONNECT_INTERVAL = 1.0

#: Errors that mean the bus is gone rather than that a value is unavailable.
#: ``serial.SerialException`` is an ``OSError``, which is what a yanked USB
#: adapter raises.
BUS_ERRORS = (FeetechError, OSError)

#: Byte sequences the Return key can produce. Terminals send CR; the line
#: discipline turns it into LF when ICRNL is set, which readchar leaves alone.
ENTER_KEYS = (readchar.key.ENTER, "\r", "\n")


class KeyListener(threading.Thread):
    """Read key presses in the background.

    Keys are queued rather than kept in a single slot: a held down arrow key
    repeats faster than the main loop polls, and dropping the repeats makes
    the screen feel stuck.
    """

    def __init__(self):
        super().__init__()
        self.keys = queue.Queue()
        self.running = True

    def run(self):
        """Queue key presses until the listener is stopped."""
        while self.running:
            try:
                key = readchar.readkey()
            except KeyboardInterrupt:
                self.keys.put("q")
                self.stop()
                break
            except Exception:
                # stdin closed underneath us, nothing left to read.
                self.stop()
                break
            self.keys.put(key)
            if key == "q":
                self.stop()
                break

    def get_key(self):
        """Take the next key press.

        Returns
        -------
        str or None
            The key, or ``None`` when the queue is empty.
        """
        try:
            return self.keys.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        """Ask the listener to finish."""
        self.running = False


def clear_screen():
    """Clear the terminal and move the cursor to the top left."""
    sys.stdout.write("\033[H\033[J")
    sys.stdout.flush()


def _enabled(flag):
    """Render a boolean as a coloured Enabled/Disabled label.

    Parameters
    ----------
    flag : bool
        Value to render.

    Returns
    -------
    str
        Coloured label.
    """
    colour = Fore.GREEN if flag else Fore.RED
    text = "Enabled" if flag else "Disabled"
    return f"{colour}{text}{Style.RESET_ALL}"


class ServoTui:
    """Terminal UI driving a :class:`~feetech_cli.controller.FeetechServoController`.

    Parameters
    ----------
    controller : FeetechServoController
        An open controller.
    servo_ids : list of int
        Servos discovered on the bus.
    id_candidates : list of int or None, optional
        Ids offered when changing a servo id. Defaults to the controller's
        configured scan range.
    """

    def __init__(self, controller, servo_ids, id_candidates=None):
        self.controller = controller
        self.servo_ids = list(servo_ids)
        if id_candidates is None:
            id_candidates = list(
                range(controller.min_id, controller.max_id + 1)
            )
        self.id_candidates = id_candidates
        self.active_index = 0
        self.selected_row = 0
        self.candidate_index = 0
        self.message = ""
        self.values = {}
        #: ``(question, action)`` while a destructive key waits for 'y'.
        self.pending_confirm = None
        #: False once the bus stops answering, until a servo is found again.
        self.connected = True

    @property
    def servo_id(self):
        """int: Id of the servo currently being controlled."""
        return self.servo_ids[self.active_index]

    def sync_from_servo(self):
        """Point the id cursor at the active servo and refresh the readings."""
        if self.servo_id in self.id_candidates:
            self.candidate_index = self.id_candidates.index(self.servo_id)
        self.values = self.read_values()

    @property
    def torque_is_on(self):
        """bool: Whether the active servo currently holds its position."""
        return bool(self.values.get("torque_enable"))

    def run(self):
        """Run the UI until the user presses ``q``.

        The servo is polled on a timer rather than once per loop, so that
        holding down an arrow key does not flood the bus, and the screen is
        repainted only when something actually changed.
        """
        listener = KeyListener()
        listener.daemon = True
        listener.start()
        if self.connected:
            try:
                self.sync_from_servo()
            except BUS_ERRORS as error:
                self.lose_bus(error)
        else:
            # Started on a quiet bus rather than having lost one.
            self.message = "Waiting for a servo to appear."

        clear_screen()
        values = {}
        last_read = None
        last_probe = None
        probe_now = False
        dirty = True
        quit_requested = False
        try:
            while not quit_requested:
                now = time.monotonic()
                if self.connected:
                    if last_read is None or now - last_read >= REFRESH_INTERVAL:
                        try:
                            values = self.read_values()
                        except BUS_ERRORS as error:
                            self.lose_bus(error)
                            values = {}
                            last_probe = None
                        last_read = now
                        dirty = True
                elif (last_probe is None
                      or now - last_probe >= RECONNECT_INTERVAL
                      or probe_now):
                    # A servo was swapped or the adapter was replugged. Keep
                    # looking, reopening the port if it went away.
                    if self.rediscover():
                        self.connected = True
                        values = self.values
                        last_read = now
                    last_probe = time.monotonic()
                    probe_now = False
                    dirty = True
                if dirty:
                    self.render(values)
                    dirty = False

                # Drain every key that arrived since the last frame, so that a
                # held down arrow key is not thrown away.
                handled = False
                while True:
                    key = listener.get_key()
                    if key is None:
                        break
                    if key == "q":
                        quit_requested = True
                        break
                    try:
                        self.handle_key(key)
                    except BUS_ERRORS as error:
                        self.lose_bus(error)
                        last_probe = None
                    handled = True
                if quit_requested:
                    break
                if handled:
                    # Show the effect of the key presses straight away.
                    last_read = None
                    if not self.connected:
                        probe_now = True
                    dirty = True
                    continue
                if not listener.running:
                    break
                time.sleep(POLL_INTERVAL)
        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()
        print("Exiting.")

    def lose_bus(self, error):
        """Record that the bus stopped answering.

        Parameters
        ----------
        error : Exception
            What the failed read raised.
        """
        self.connected = False
        self.values = {}
        self.pending_confirm = None
        self.message = f"{Fore.RED}Lost the servo: {error}{Style.RESET_ALL}"
        if isinstance(error, OSError):
            # The port itself went away, so it has to be opened again rather
            # than just rescanned. A swapped servo leaves the port alone.
            self.controller.close()

    def rediscover(self):
        """Look for whatever is on the bus now.

        Reopens the port when it was lost, rescans the id range, and moves to
        the servo that is there. This is what makes swapping a servo while the
        screen is running work.

        Returns
        -------
        bool
            ``True`` when a servo is answering again.
        """
        try:
            if not self.controller.is_open:
                self.controller.open()
            found = self.controller.scan()
        except BUS_ERRORS:
            self.controller.close()
            return False
        if not found:
            return False

        previous = self.servo_ids[self.active_index] if self.servo_ids else None
        self.servo_ids = found
        self.active_index = found.index(previous) if previous in found else 0
        try:
            self.sync_from_servo()
        except BUS_ERRORS:
            return False
        ids = " ".join(str(sid) for sid in found)
        if previous in found:
            self.message = f"{Fore.GREEN}Reconnected to servo {previous}.{Style.RESET_ALL}"
        else:
            self.message = (
                f"{Fore.GREEN}Found servo ids: {ids}. Now on {self.servo_id}."
                f"{Style.RESET_ALL}"
            )
        return True

    def disconnected_lines(self):
        """Build the screen shown while nothing is answering.

        Returns
        -------
        list of str
            The lines to paint.
        """
        low = self.controller.min_id
        high = self.controller.max_id
        return [
            self.title(),
            "",
            f"{Fore.RED}{Style.BRIGHT}No servo is answering.{Style.RESET_ALL}",
            "",
            "Plug a servo in and it will be picked up automatically.",
            "Retrying once a second. Also worth checking:",
            "  - the USB adapter is still connected",
            "  - the servo has its own power supply, USB alone is not enough",
            f"  - the servo id is between {low} and {high} "
            f"(widen with --min-id / --max-id)",
            "",
            self.message,
            "",
            "'r' retry now, 'q' quit",
        ]

    def read_values(self):
        """Read the registers shown on screen.

        Returns
        -------
        dict
            Maps register name to value, with ``None`` for any register that
            could not be read.
        """
        values = self.controller.dump(self.servo_id, STATUS_REGISTERS)
        if all(value is None for value in values.values()):
            # dump() turns a per register failure into None so that one odd
            # register cannot blank the screen. Everything failing means the
            # servo is gone, which the caller has to know about.
            raise FeetechTimeoutError(
                f"servo {self.servo_id} stopped answering"
            )
        self.values = values
        return values

    def build_lines(self, values):
        """Build the screen as a list of lines.

        Parameters
        ----------
        values : dict
            Register values as returned by :meth:`read_values`.

        Returns
        -------
        list of str
            The lines to paint, without trailing newlines.
        """
        servo_id = self.servo_id
        bus = " ".join(
            f"{Fore.CYAN}{sid}{Style.RESET_ALL}" if sid == servo_id else str(sid)
            for sid in self.servo_ids
        )
        if not self.connected:
            return self.disconnected_lines()
        selected_row = SELECTABLE_ROWS[self.selected_row]
        speed = format_baud(self.controller.baudrate)
        lines = [
            self.title(),
            f"Bus: {self.controller.serial.port} @ {speed}bps   Servos: {bus}",
            "",
        ]
        for index, row in enumerate(SELECTABLE_ROWS):
            text = self.render_row(row, values)
            if index == self.selected_row:
                lines.append(
                    f"{Fore.CYAN}{Style.BRIGHT}>> {row}{Style.RESET_ALL}: {text}"
                )
            else:
                lines.append(f"   {row}: {text}")
        lines.append("")
        for row in STATUS_ROWS:
            lines.append(f"    {row}: {self.render_row(row, values)}")
        lines += [
            "",
            "----------------------",
            "Up/Down move between the four rows marked "
            f"{Fore.CYAN}{Style.BRIGHT}>>{Style.RESET_ALL} above",
            "Now on {}{}{}: {}".format(
                Fore.CYAN + Style.BRIGHT,
                selected_row,
                Style.RESET_ALL,
                ROW_HINTS.get(selected_row, ""),
            ),
            "'n'/'p' switch to the next/previous servo on the bus, 'r' rescan",
            "'t' toggle torque, 'z' move to 0 deg, 'm' cycle operating mode",
            "'0' call the pose the servo is in right now 0 deg (writes EEPROM)",
            "'q' quit",
        ]
        if self.pending_confirm is not None:
            lines += [
                "",
                f"{Fore.YELLOW}{Style.BRIGHT}{self.pending_confirm[0]}"
                f"{Style.RESET_ALL}",
            ]
        elif self.message:
            lines += ["", self.message]
        return lines

    def title(self):
        """Return the header line.

        A simulated bus says so on every frame. The banner printed before the
        screen starts is wiped by the first repaint, which left no way to tell
        a simulation from real hardware.

        Returns
        -------
        str
            The header line.
        """
        if getattr(self.controller, "is_simulated", False):
            return (
                f"{Fore.YELLOW}{Style.BRIGHT}--- SIMULATED SERVOS, "
                f"NO HARDWARE ---{Style.RESET_ALL}"
            )
        return "--- FEETECH Servo Status ---"

    def render(self, values):
        """Paint the screen in place.

        The cursor is moved home and every line is overwritten and cleared to
        the end, rather than blanking the whole screen first. Clearing first
        makes the display flicker on every frame.

        Parameters
        ----------
        values : dict
            Register values as returned by :meth:`read_values`.
        """
        chunks = ["\033[H"]
        for line in self.build_lines(values):
            chunks.append(line)
            chunks.append("\033[K\n")
        # Wipe whatever the previous, longer frame left below.
        chunks.append("\033[J")
        sys.stdout.write("".join(chunks))
        sys.stdout.flush()

    def draw(self):
        """Read the servo and paint one frame."""
        self.render(self.read_values())

    def render_row(self, row, values):
        """Render one status row.

        Parameters
        ----------
        row : str
            Row label.
        values : dict
            Register values as returned by
            :meth:`~feetech_cli.controller.FeetechServoController.dump`.

        Returns
        -------
        str
            The text after the label.
        """
        if row == "Servo ID":
            text = str(self.servo_id)
            if self.selected_row == SELECTABLE_ROWS.index("Servo ID"):
                choices = []
                for index, candidate in enumerate(self.id_candidates):
                    if index == self.candidate_index:
                        choices.append(
                            f"{Fore.GREEN}{candidate}{Style.RESET_ALL}"
                        )
                    else:
                        choices.append(str(candidate))
                text += "  ->  new id: [{}]".format(" ".join(choices))
            return text
        if row == "Goal Position":
            goal = values.get("goal_position")
            if goal is None:
                return "no data"
            text = f"{goal} ({position_to_degree(goal):+.1f} deg)"
            if not self.torque_is_on:
                # Measured on an STS3215: a goal_position write switches the
                # output stage on by itself, so the arrows move the servo even
                # from a standstill. Saying "torque is off so it will not move"
                # here would be wrong, and would read as safe when it is not.
                text += (
                    f"  {Fore.YELLOW}(arrows will energise the servo)"
                    f"{Style.RESET_ALL}"
                )
            return text
        if row == "Goal Velocity":
            return _plain(values.get("goal_velocity"))
        if row == "Acceleration":
            return _plain(values.get("acceleration"))
        if row == "Present Position":
            position = values.get("present_position")
            if position is None:
                return "no data"
            text = f"{position} ({position_to_degree(position):+.1f} deg)"
            if self.torque_is_on:
                # The counterpart of the goal row's warning: an energised servo
                # springs back when pushed, so hand movement does not stick and
                # this row looks frozen.
                text += (
                    f"  {Fore.YELLOW}(held by the servo - press 't' to move "
                    f"it by hand){Style.RESET_ALL}"
                )
            return text
        if row == "Present Velocity":
            return _plain(values.get("present_velocity"))
        if row == "Present Load":
            load = values.get("present_load")
            if load is None:
                return "no data"
            return f"{load} ({load / 10.0:+.1f}%)"
        if row == "Voltage":
            voltage = values.get("present_voltage")
            if voltage is None:
                return "no data"
            return f"{voltage / 10.0:.1f} V"
        if row == "Temperature":
            temperature = values.get("present_temperature")
            if temperature is None:
                return "no data"
            colour = Fore.RED if temperature >= 60 else Fore.GREEN
            return f"{colour}{temperature} C{Style.RESET_ALL}"
        if row == "Current":
            current = values.get("present_current")
            if current is None:
                return "no data"
            return f"{current} ({current * 6.5:.0f} mA)"
        if row == "Torque":
            torque = values.get("torque_enable")
            if torque is None:
                return "no data"
            return _enabled(bool(torque))
        if row == "Operating Mode":
            mode = values.get("operating_mode")
            if mode is None:
                return "no data"
            return "{} ({})".format(OPERATING_MODES.get(mode, "unknown"), mode)
        if row == "Moving":
            moving = values.get("moving")
            if moving is None:
                return "no data"
            return "yes" if moving else "no"
        if row == "Position Limits":
            low = values.get("min_position_limit")
            high = values.get("max_position_limit")
            if low is None or high is None:
                return "no data"
            return f"{low} .. {high}"
        if row == "Homing Offset":
            return _plain(values.get("homing_offset"))
        if row == "Baud Rate":
            index = values.get("baud_rate")
            if index is None:
                return "no data"
            baud = BAUDRATE_TABLE.get(index)
            if baud is None:
                return f"unknown index {index}"
            return f"{format_baud(baud)}bps"
        if row == "EEPROM":
            lock = values.get("lock")
            if lock is None:
                return "no data"
            return "locked" if lock else f"{Fore.YELLOW}unlocked{Style.RESET_ALL}"
        return "no data"

    def handle_key(self, key):
        """Act on one key press.

        Parameters
        ----------
        key : str
            Key returned by :meth:`KeyListener.get_key`.
        """
        if not self.connected:
            if key == "r":
                self.message = "Looking for a servo..."
            return

        if self.pending_confirm is not None:
            _, action = self.pending_confirm
            self.pending_confirm = None
            if key == "y":
                action()
            else:
                self.message = "Cancelled."
            return

        self.message = ""
        row = SELECTABLE_ROWS[self.selected_row]

        if key == readchar.key.UP:
            self.selected_row = (self.selected_row - 1) % len(SELECTABLE_ROWS)
        elif key == readchar.key.DOWN:
            self.selected_row = (self.selected_row + 1) % len(SELECTABLE_ROWS)
        elif key == readchar.key.LEFT:
            self.adjust(row, -1)
        elif key == readchar.key.RIGHT:
            self.adjust(row, 1)
        elif key in ENTER_KEYS and row == "Servo ID":
            self.apply_id()
        elif key == "n":
            self.active_index = (self.active_index + 1) % len(self.servo_ids)
            self.sync_from_servo()
        elif key == "p":
            self.active_index = (self.active_index - 1) % len(self.servo_ids)
            self.sync_from_servo()
        elif key == "r":
            self.rescan()
        elif key == "t":
            if self.torque_is_on:
                self.controller.set_torque(self.servo_id, False)
                self.values["torque_enable"] = 0
                self.message = "Torque disabled, the horn moves freely."
            else:
                self.enable_torque()
        elif key == "z":
            self.controller.set_goal_position(self.servo_id, CENTER_POSITION)
            self.warn_if_torque_off()
        elif key == "0":
            self.ask_to_zero()
        elif key == "m":
            current = self.controller.read_register(self.servo_id, "operating_mode")
            modes = sorted(OPERATING_MODES)
            nxt = modes[(modes.index(current) + 1) % len(modes)] if current in modes else 0
            self.controller.set_operating_mode(self.servo_id, nxt)
            self.message = f"Operating mode set to {OPERATING_MODES[nxt]}"

    def adjust(self, row, direction):
        """Change the value of the selected row.

        Parameters
        ----------
        row : str
            Row label.
        direction : int
            ``-1`` for left, ``+1`` for right.
        """
        if row == "Servo ID":
            self.candidate_index = (
                self.candidate_index + direction
            ) % len(self.id_candidates)
        elif row == "Goal Position":
            target = self.step_base() + direction * POSITION_STEP
            target = max(0, min(RESOLUTION - 1, target))
            self.controller.set_goal_position(self.servo_id, target)
            self.values["goal_position"] = target
            self.warn_if_torque_off()
        elif row == "Goal Velocity":
            current = self.values.get("goal_velocity") or 0
            target = max(
                -RESOLUTION, min(RESOLUTION, current + direction * VELOCITY_STEP)
            )
            self.controller.set_goal_velocity(self.servo_id, target)
            self.values["goal_velocity"] = target
            self.warn_if_torque_off()
        elif row == "Acceleration":
            current = self.values.get("acceleration") or 0
            self.values["acceleration"] = self.controller.set_acceleration(
                self.servo_id, current + direction * ACCELERATION_STEP
            )

    def step_base(self):
        """Return the position an arrow key press should move away from.

        Steps run from the commanded position so that repeated presses
        accumulate. The risk that comes with that, a stale goal making the
        servo snap when it is energised, is handled in
        :meth:`enable_torque`, not by refusing to accumulate here.

        Returns
        -------
        int
            The encoder count to step from.
        """
        for name in ("goal_position", "present_position"):
            base = self.values.get(name)
            if base is not None:
                return base
        return self.controller.read_register(self.servo_id, "present_position")

    def enable_torque(self):
        """Energise the servo so that it holds where it is now.

        The goal register is set to the measured position first. Without that
        the servo would jump to whatever was last commanded the instant the
        output stage came on, which on an assembled robot is a real hazard.
        """
        servo_id = self.servo_id
        position = self.controller.read_register(servo_id, "present_position")
        self.controller.set_goal_position(servo_id, position)
        self.controller.set_torque(servo_id, True)
        self.values["goal_position"] = position
        self.values["torque_enable"] = 1
        self.message = (
            f"{Fore.GREEN}Torque enabled, holding the current position "
            f"({position}).{Style.RESET_ALL}"
        )

    def warn_if_torque_off(self):
        """Say that a motion command has just energised a limp servo.

        Writing ``goal_position`` turns the output stage on by itself, so a
        servo that was limp starts holding as soon as an arrow is pressed.
        Worth announcing: the horn stops being back drivable at that moment.
        """
        if not self.torque_is_on:
            self.message = (
                f"{Fore.YELLOW}That command energised the servo. Press 't' to "
                f"make the joint free to move by hand again.{Style.RESET_ALL}"
            )

    def ask_to_zero(self):
        """Ask before calling the current pose zero degrees."""
        present = self.values.get("present_position")
        if present is None:
            self.message = f"{Fore.RED}No position reading yet.{Style.RESET_ALL}"
            return
        self.pending_confirm = (
            f"Call this pose ({present}, {position_to_degree(present):+.1f} deg) "
            "0 degrees? This writes homing_offset to the servo EEPROM. "
            "The servo will not move. "
            "Press 'y' to confirm, any other key to cancel.",
            self.apply_zero,
        )

    def apply_zero(self):
        """Re-label the current pose as zero degrees."""
        reported = self.controller.set_zero(self.servo_id)
        self.values = self.read_values()
        self.message = (
            f"{Fore.GREEN}This pose is now {reported} "
            f"({position_to_degree(reported):+.1f} deg).{Style.RESET_ALL}"
        )

    def apply_id(self):
        """Write the highlighted candidate id to the active servo."""
        new_id = self.id_candidates[self.candidate_index]
        old_id = self.servo_id
        if new_id == old_id:
            self.message = f"Servo is already at id {new_id}"
            return
        confirmed = self.controller.set_id(old_id, new_id)
        self.servo_ids[self.active_index] = confirmed
        self.servo_ids = sorted(set(self.servo_ids))
        self.active_index = self.servo_ids.index(confirmed)
        self.message = f"{Fore.GREEN}Servo {old_id} is now id {confirmed}{Style.RESET_ALL}"

    def rescan(self):
        """Probe the bus again and refresh the list of known servos."""
        found = self.controller.scan()
        if not found:
            self.message = f"{Fore.RED}No servo answered on the bus{Style.RESET_ALL}"
            return
        current = self.servo_id
        self.servo_ids = found
        self.active_index = found.index(current) if current in found else 0
        self.sync_from_servo()
        self.message = "Found servo ids: {}".format(
            " ".join(str(sid) for sid in found)
        )


def _plain(value):
    """Render an optional integer.

    Parameters
    ----------
    value : int or None
        Value to render.

    Returns
    -------
    str
        The value, or ``"no data"`` when it is ``None``.
    """
    if value is None:
        return "no data"
    return str(value)
