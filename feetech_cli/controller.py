"""High level control of FEETECH serial bus servos."""

import time

import serial
import serial.tools.list_ports

from feetech_cli.protocol import BROADCAST_ID
from feetech_cli.protocol import FeetechError
from feetech_cli.protocol import PacketHandler
from feetech_cli.protocol import describe_error
from feetech_cli.protocol import from_sign_magnitude
from feetech_cli.protocol import to_sign_magnitude
from feetech_cli.registers import BAUDRATE_INDEX
from feetech_cli.registers import BAUDRATE_TABLE
from feetech_cli.registers import CENTER_POSITION
from feetech_cli.registers import CONTROL_TABLE
from feetech_cli.registers import DEFAULT_MAX_ID
from feetech_cli.registers import DEFAULT_MIN_ID
from feetech_cli.registers import EEPROM_LOCKED
from feetech_cli.registers import EEPROM_UNLOCKED
from feetech_cli.registers import MAX_HOMING_OFFSET
from feetech_cli.registers import OPERATING_MODES
from feetech_cli.registers import RESOLUTION
from feetech_cli.registers import SCAN_BAUDRATES

#: USB serial bridges commonly found on FEETECH servo adapters, as
#: ``(vendor id, product id)`` pairs. Used only to rank candidate ports.
KNOWN_ADAPTERS = [
    (0x1A86, 0x7523),  # CH340, FE-URT-1 and many clones
    (0x1A86, 0x55D3),  # CH343/CH9102, Waveshare bus servo driver
    (0x1A86, 0x55D4),  # CH9102F
    (0x10C4, 0xEA60),  # CP2102
    (0x0403, 0x6001),  # FT232R
    (0x0403, 0x6015),  # FT231X
    (0x2341, 0x0043),  # Arduino based adapters
]


class PortNotFoundError(FeetechError):
    """No serial port matching the request could be opened."""


class NoServoFoundError(FeetechError):
    """No servo answered on the bus."""


def list_candidate_ports():
    """List serial ports that could carry a servo bus, best guess first.

    Ports that are obviously not USB serial adapters, such as the macOS
    Bluetooth and debug console devices, are dropped.

    Returns
    -------
    list of serial.tools.list_ports_common.ListPortInfo
        Candidate ports ordered so that known adapter chips come first.
    """
    ports = []
    for port in serial.tools.list_ports.comports():
        name = port.device.lower()
        if "bluetooth" in name or "debug-console" in name:
            continue
        if port.vid is None:
            continue
        ports.append(port)

    def rank(port):
        """Sort key placing known adapter chips first."""
        if (port.vid, port.pid) in KNOWN_ADAPTERS:
            return (0, port.device)
        return (1, port.device)

    return sorted(ports, key=rank)


class FeetechServoController:
    """Talk to FEETECH STS/SMS/SCS servos on a serial bus.

    Parameters
    ----------
    port : str or None, optional
        Serial device to open. When ``None`` the first plausible USB serial
        adapter is used.
    baudrate : int or None, optional
        Bus speed. When ``None`` every speed in
        :data:`~feetech_cli.registers.SCAN_BAUDRATES` is probed until a servo
        answers.
    timeout : float, optional
        Serial read timeout in seconds.
    little_endian : bool, optional
        ``True`` for the STS/SMS series, ``False`` for the SCS series.
    min_id : int, optional
        Lowest servo id used when probing the bus.
    max_id : int, optional
        Highest servo id used when probing the bus.
    """

    def __init__(
        self,
        port=None,
        baudrate=None,
        timeout=0.05,
        little_endian=True,
        min_id=DEFAULT_MIN_ID,
        max_id=DEFAULT_MAX_ID,
    ):
        self.requested_port = port
        self.requested_baudrate = baudrate
        self.timeout = timeout
        self.little_endian = little_endian
        self.min_id = min_id
        self.max_id = max_id
        self.serial = None
        self.packet_handler = None
        self.baudrate = None
        #: Set by ``--simulate`` so the screen can label itself. Never set as
        #: a consequence of hardware being absent.
        self.is_simulated = False
        #: Servos whose ``response_status_level`` is 0 and which therefore
        #: never answer a write instruction.
        self.silent_servos = set()

    # -- connection --------------------------------------------------------

    @property
    def is_open(self):
        """bool: Whether the serial port is currently open."""
        return self.serial is not None and self.serial.is_open

    def open(self, require_servo=True):
        """Open the bus, detecting the port and speed when they were not given.

        Parameters
        ----------
        require_servo : bool, optional
            When ``False`` the port is left open even if nothing answers, so
            that the caller can keep watching for a servo to be plugged in.

        Returns
        -------
        list of int
            Ids of the servos that answered on the bus. Empty when nothing
            answered and ``require_servo`` is ``False``.

        Raises
        ------
        PortNotFoundError
            If no serial port could be opened.
        NoServoFoundError
            If the speed had to be detected, no servo answered at any speed,
            and ``require_servo`` is ``True``.
        """
        if self.requested_port is not None:
            devices = [self.requested_port]
        else:
            devices = [port.device for port in list_candidate_ports()]
            if not devices:
                raise PortNotFoundError(
                    "no USB serial adapter found. Pass --port to name the device "
                    "explicitly."
                )

        if self.requested_baudrate is not None:
            baudrates = [self.requested_baudrate]
        else:
            baudrates = list(SCAN_BAUDRATES)

        last_error = None
        for device in devices:
            for baudrate in baudrates:
                try:
                    self._open_port(device, baudrate)
                except (OSError, serial.SerialException) as error:
                    last_error = error
                    continue
                if self.requested_baudrate is not None:
                    # The user pinned the speed, report what is there but do
                    # not reject the port when the bus is quiet.
                    return self.scan()
                found = self.scan()
                if found:
                    return found
                self.close()

        if last_error is not None and not self.is_open:
            raise PortNotFoundError(
                "could not open {}: {}".format(", ".join(devices), last_error)
            )
        if not require_servo:
            # Hold the port open at the first speed that works, so a servo
            # plugged in later can be found without reopening anything.
            for device in devices:
                try:
                    self._open_port(device, baudrates[0])
                    return []
                except (OSError, serial.SerialException) as error:
                    last_error = error
                    continue
            raise PortNotFoundError(
                "could not open {}: {}".format(", ".join(devices), last_error)
            )
        raise NoServoFoundError(
            "no servo answered on {} at any of {}".format(
                ", ".join(devices),
                ", ".join(str(baud) for baud in baudrates),
            )
        )

    def _open_port(self, device, baudrate):
        """Open one serial device at one speed.

        Parameters
        ----------
        device : str
            Serial device path.
        baudrate : int
            Bus speed.
        """
        self.close()
        self.serial = serial.Serial(device, baudrate, timeout=self.timeout)
        self.baudrate = baudrate
        self.packet_handler = PacketHandler(
            self.serial, little_endian=self.little_endian
        )
        self.silent_servos = set()

    def reopen(self, baudrate):
        """Reopen the current port at a different speed.

        Parameters
        ----------
        baudrate : int
            New bus speed.
        """
        if not self.is_open:
            raise FeetechError("port is not open")
        device = self.serial.port
        self._open_port(device, baudrate)

    def close(self):
        """Close the serial port if it is open."""
        if self.serial is not None and self.serial.is_open:
            self.serial.close()
        self.serial = None
        self.packet_handler = None

    def __enter__(self):
        """Open the bus on entering a ``with`` block.

        Returns
        -------
        FeetechServoController
            This controller.
        """
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Close the bus on leaving a ``with`` block."""
        self.close()
        return False

    def _require_open(self):
        """Raise unless the serial port is open."""
        if not self.is_open:
            raise FeetechError("serial port is not open")

    # -- discovery ---------------------------------------------------------

    def ping(self, servo_id):
        """Check whether one servo answers.

        Parameters
        ----------
        servo_id : int
            Servo id to probe.

        Returns
        -------
        bool
            ``True`` if the servo answered.
        """
        self._require_open()
        return self.packet_handler.ping(servo_id)

    def scan(self, min_id=None, max_id=None, progress=None):
        """Probe a range of ids and report which servos answer.

        Parameters
        ----------
        min_id : int or None, optional
            Lowest id to probe. Defaults to the controller's ``min_id``.
        max_id : int or None, optional
            Highest id to probe. Defaults to the controller's ``max_id``.
        progress : callable or None, optional
            Called as ``progress(servo_id, found)`` after each probe.

        Returns
        -------
        list of int
            Ids that answered, in ascending order.
        """
        self._require_open()
        if min_id is None:
            min_id = self.min_id
        if max_id is None:
            max_id = self.max_id
        found = []
        for servo_id in range(min_id, max_id + 1):
            if servo_id == BROADCAST_ID:
                continue
            answered = self.packet_handler.ping(servo_id)
            if answered:
                found.append(servo_id)
            if progress is not None:
                progress(servo_id, answered)
        return found

    def scan_all_baudrates(self, min_id=None, max_id=None, progress=None):
        """Probe every supported bus speed and report the servos found.

        The port is left open at the speed of the last probe that found
        something, or at the original speed when nothing was found.

        Parameters
        ----------
        min_id : int or None, optional
            Lowest id to probe.
        max_id : int or None, optional
            Highest id to probe.
        progress : callable or None, optional
            Called as ``progress(baudrate, found_ids)`` after each speed.

        Returns
        -------
        dict
            Maps bus speed to the list of ids that answered at that speed.
            Speeds where nothing answered are omitted.
        """
        self._require_open()
        original_baudrate = self.baudrate
        results = {}
        best_baudrate = None
        for baudrate in SCAN_BAUDRATES:
            self.reopen(baudrate)
            found = self.scan(min_id=min_id, max_id=max_id)
            if progress is not None:
                progress(baudrate, found)
            if found:
                results[baudrate] = found
                best_baudrate = baudrate
        self.reopen(best_baudrate or original_baudrate)
        return results

    # -- register access ---------------------------------------------------

    def read_register(self, servo_id, name):
        """Read a named register.

        Parameters
        ----------
        servo_id : int
            Servo id.
        name : str
            Register name, a key of
            :data:`~feetech_cli.registers.CONTROL_TABLE`.

        Returns
        -------
        int
            The register value, sign decoded where applicable.
        """
        self._require_open()
        register = _lookup(name)
        data = self.packet_handler.read(servo_id, register.address, register.size)
        value = self.packet_handler.decode(register.size, data)
        if register.sign_bit is not None:
            return from_sign_magnitude(value, register.sign_bit)
        return value

    def write_register(self, servo_id, name, value):
        """Write a named register.

        EEPROM registers are unlocked before the write and locked again
        afterwards. The value is read back and compared, so a silently
        rejected write raises instead of appearing to succeed.

        Parameters
        ----------
        servo_id : int
            Servo id.
        name : str
            Register name.
        value : int
            Value to write, sign encoded where applicable.

        Returns
        -------
        int
            The value read back from the servo.
        """
        self._require_open()
        register = _lookup(name)
        if not register.writable:
            raise FeetechError(f"register '{name}' is read only")
        if register.area == "eeprom":
            self.unlock_eeprom(servo_id)
        try:
            self._write_raw(servo_id, register, value)
        finally:
            if register.area == "eeprom":
                self.lock_eeprom(servo_id)
        return self.read_register(servo_id, name)

    def _write_raw(self, servo_id, register, value, expect_status=None):
        """Write one register without touching the EEPROM lock.

        Parameters
        ----------
        servo_id : int
            Servo id.
        register : Register
            Control table entry.
        value : int
            Value to write.
        expect_status : bool or None, optional
            Whether to wait for the status packet. ``None`` decides from
            :attr:`silent_servos`. Pass ``False`` for writes that change how
            the servo is addressed, such as ``id`` and ``baud_rate``: after
            those the servo may answer under its new identity or at the new
            bus speed, so the answer cannot be matched reliably and the change
            is confirmed by reading it back instead.
        """
        if register.sign_bit is not None:
            value = to_sign_magnitude(value, register.sign_bit)
        if expect_status is None:
            expect_status = servo_id not in self.silent_servos
        data = self.packet_handler.encode(register.size, value)
        self.packet_handler.write(
            servo_id, register.address, data, expect_status=expect_status,
        )

    def unlock_eeprom(self, servo_id):
        """Allow EEPROM writes on a servo.

        Parameters
        ----------
        servo_id : int
            Servo id.
        """
        self._write_raw(servo_id, _lookup("lock"), EEPROM_UNLOCKED)
        time.sleep(0.02)

    def lock_eeprom(self, servo_id):
        """Write protect the EEPROM of a servo.

        Parameters
        ----------
        servo_id : int
            Servo id.
        """
        self._write_raw(servo_id, _lookup("lock"), EEPROM_LOCKED)
        time.sleep(0.02)

    def detect_response_level(self, servo_id):
        """Record whether a servo answers write instructions.

        Servos with ``response_status_level`` set to 0 answer ``PING`` and
        ``READ`` but stay silent on ``WRITE``. Writes to those servos must not
        wait for a status packet.

        Parameters
        ----------
        servo_id : int
            Servo id.

        Returns
        -------
        int
            The ``response_status_level`` value that was read.
        """
        level = self.read_register(servo_id, "response_status_level")
        if level == 0:
            self.silent_servos.add(servo_id)
        else:
            self.silent_servos.discard(servo_id)
        return level

    def dump(self, servo_id, names=None):
        """Read several registers at once.

        Parameters
        ----------
        servo_id : int
            Servo id.
        names : sequence of str or None, optional
            Register names to read. Defaults to every entry of the control
            table.

        Returns
        -------
        dict
            Maps register name to value. A register that could not be read is
            mapped to ``None``.
        """
        if names is None:
            names = list(CONTROL_TABLE)
        result = {}
        for name in names:
            try:
                result[name] = self.read_register(servo_id, name)
            except FeetechError:
                result[name] = None
        return result

    # -- configuration -----------------------------------------------------

    def set_id(self, servo_id, new_id):
        """Change the id of a servo.

        Torque is switched off first, because a servo that is actively holding
        a position can reject EEPROM writes.

        Parameters
        ----------
        servo_id : int
            Current servo id.
        new_id : int
            Desired servo id, 0-253.

        Returns
        -------
        int
            The id read back from the servo after the change.

        Raises
        ------
        FeetechError
            If ``new_id`` is out of range, is already taken by another servo
            on the bus, or if the servo does not answer at the new id.
        """
        self._require_open()
        if not 0 <= new_id <= 253:
            raise FeetechError(
                f"servo id must be between 0 and 253, got {new_id}"
            )
        if new_id == servo_id:
            return servo_id
        if self.ping(new_id):
            raise FeetechError(
                f"id {new_id} is already used by another servo on this bus"
            )
        if not self.ping(servo_id):
            raise FeetechError(f"no servo answered at id {servo_id}")

        self.set_torque(servo_id, False)
        self.unlock_eeprom(servo_id)
        try:
            self._write_raw(servo_id, _lookup("id"), new_id, expect_status=False)
            time.sleep(0.05)
        finally:
            # The servo answers on the new id from here on, so the EEPROM has
            # to be locked again through the new address.
            target = new_id if self.ping(new_id) else servo_id
            self.lock_eeprom(target)
            if servo_id in self.silent_servos:
                self.silent_servos.discard(servo_id)
                self.silent_servos.add(target)

        if not self.ping(new_id):
            raise FeetechError(
                f"servo did not answer at the new id {new_id}. It may still be at id "
                f"{servo_id}; power cycle and rescan the bus."
            )
        return self.read_register(new_id, "id")

    def set_baudrate(self, servo_id, baudrate):
        """Change the bus speed a servo uses.

        The serial port is reopened at the new speed and the servo is pinged
        to confirm the change actually took effect.

        Parameters
        ----------
        servo_id : int
            Servo id.
        baudrate : int
            Desired bus speed, a key of
            :data:`~feetech_cli.registers.BAUDRATE_INDEX`.

        Returns
        -------
        int
            The bus speed read back from the servo.

        Raises
        ------
        FeetechError
            If the speed is not supported or the servo does not answer at the
            new speed.
        """
        self._require_open()
        if baudrate not in BAUDRATE_INDEX:
            raise FeetechError(
                "unsupported baud rate {}. Supported: {}".format(
                    baudrate, ", ".join(str(rate) for rate in sorted(BAUDRATE_INDEX))
                )
            )
        if not self.ping(servo_id):
            raise FeetechError(f"no servo answered at id {servo_id}")

        index = BAUDRATE_INDEX[baudrate]
        self.set_torque(servo_id, False)
        self.unlock_eeprom(servo_id)
        try:
            self._write_raw(
                servo_id, _lookup("baud_rate"), index, expect_status=False
            )
            time.sleep(0.05)
        finally:
            self.reopen(baudrate)
            if self.ping(servo_id):
                self.lock_eeprom(servo_id)

        if not self.ping(servo_id):
            raise FeetechError(
                f"servo {servo_id} did not answer at {baudrate} baud after the change. Rescan "
                "the bus with 'feetech scan --all-baudrates'."
            )
        return BAUDRATE_TABLE[self.read_register(servo_id, "baud_rate")]

    def factory_reset(self, servo_id):
        """Restore the factory defaults of a servo.

        The servo returns to id 1 at 1000000 baud.

        Parameters
        ----------
        servo_id : int
            Servo id.
        """
        self._require_open()
        self.set_torque(servo_id, False)
        self.packet_handler.factory_reset(
            servo_id, expect_status=servo_id not in self.silent_servos
        )
        time.sleep(0.1)

    # -- motion ------------------------------------------------------------

    def set_torque(self, servo_id, enable):
        """Enable or disable the output stage of a servo.

        Parameters
        ----------
        servo_id : int
            Servo id.
        enable : bool
            ``True`` to hold position, ``False`` to let the horn move freely.

        Returns
        -------
        int
            The ``torque_enable`` value read back.
        """
        self._write_raw(servo_id, _lookup("torque_enable"), 1 if enable else 0)
        time.sleep(0.01)
        return self.read_register(servo_id, "torque_enable")

    def set_goal_position(self, servo_id, position):
        """Command a servo to move to an encoder position.

        Parameters
        ----------
        servo_id : int
            Servo id.
        position : int
            Target encoder count.

        Returns
        -------
        int
            The commanded position.
        """
        position = int(position)
        self._write_raw(servo_id, _lookup("goal_position"), position)
        return position

    def sync_write_positions(self, servo_ids, positions, velocity=0,
                             acceleration=0):
        """Command many servos to new positions in a single packet.

        Mirrors the vendor SDK's ``SyncWritePosEx``: acceleration, goal
        position, goal time and goal speed are contiguous from address 41, so
        one broadcast covers all four.

        Nothing is read back. A sync write is a broadcast, so the servos stay
        silent by design; verify with a separate read when it matters.

        Parameters
        ----------
        servo_ids : sequence of int
            Servos to command.
        positions : sequence of int
            Target encoder counts, in the same order as ``servo_ids``.
        velocity : int, optional
            Goal speed. 0 means the servo's maximum.
        acceleration : int, optional
            Acceleration ramp, 0-254.
        """
        self._require_open()
        handler = self.packet_handler
        entries = []
        for servo_id, position in zip(servo_ids, positions):
            encoded = to_sign_magnitude(
                int(position), CONTROL_TABLE["goal_position"].sign_bit
            )
            entries.append((
                servo_id,
                [max(0, min(254, int(acceleration)))]
                + handler.encode_word(encoded)
                + handler.encode_word(0)                       # goal time
                + handler.encode_word(max(0, int(velocity))),  # goal speed
            ))
        handler.sync_write(CONTROL_TABLE["acceleration"].address, entries)

    def set_goal_velocity(self, servo_id, velocity):
        """Set the target velocity.

        In position mode this caps the travel speed; in wheel mode it drives
        the servo continuously.

        Parameters
        ----------
        servo_id : int
            Servo id.
        velocity : int
            Target velocity in encoder counts per second, signed.

        Returns
        -------
        int
            The commanded velocity.
        """
        velocity = int(velocity)
        self._write_raw(servo_id, _lookup("goal_velocity"), velocity)
        return velocity

    def set_acceleration(self, servo_id, acceleration):
        """Set the acceleration ramp.

        Parameters
        ----------
        servo_id : int
            Servo id.
        acceleration : int
            Acceleration value, 0-254.

        Returns
        -------
        int
            The commanded acceleration.
        """
        acceleration = max(0, min(254, int(acceleration)))
        self._write_raw(servo_id, _lookup("acceleration"), acceleration)
        return acceleration

    def wait_until_stopped(self, servo_id, timeout=3.0, tolerance=3):
        """Block until the servo stops moving.

        Parameters
        ----------
        servo_id : int
            Servo id.
        timeout : float, optional
            Give up after this many seconds and report where the servo is.
        tolerance : int, optional
            Encoder counts of movement between samples still counted as still.

        Returns
        -------
        int
            The position the servo came to rest at.
        """
        deadline = time.monotonic() + timeout
        last = self.read_register(servo_id, "present_position")
        still_since = None
        while time.monotonic() < deadline:
            time.sleep(0.05)
            now = self.read_register(servo_id, "present_position")
            moving = self.read_register(servo_id, "moving")
            if abs(now - last) <= tolerance and not moving:
                if still_since is None:
                    still_since = time.monotonic()
                elif time.monotonic() - still_since >= 0.1:
                    return now
            else:
                still_since = None
            last = now
        return self.read_register(servo_id, "present_position")

    def preflight(self, servo_id):
        """Check the settings that stop a servo from moving at all.

        Parameters
        ----------
        servo_id : int
            Servo id.

        Returns
        -------
        list of tuple
            ``(name, ok, detail)`` for each check, in report order.
        """
        values = self.dump(
            servo_id,
            [
                "operating_mode", "torque_limit", "max_torque_limit", "status",
                "present_voltage", "min_position_limit", "max_position_limit",
                "present_temperature", "max_temperature_limit",
            ],
        )
        mode = values.get("operating_mode")
        torque_limit = values.get("torque_limit")
        max_torque = values.get("max_torque_limit")
        status = values.get("status")
        voltage = values.get("present_voltage")
        low = values.get("min_position_limit")
        high = values.get("max_position_limit")
        temperature = values.get("present_temperature")
        max_temperature = values.get("max_temperature_limit")
        return [
            ("operating mode is position", mode == 0,
             "{} ({})".format(mode, OPERATING_MODES.get(mode, "unknown"))),
            ("torque limit above zero", bool(torque_limit), str(torque_limit)),
            ("max torque limit above zero", bool(max_torque), str(max_torque)),
            ("no hardware error", status == 0,
             "0x{:02X} ({})".format(status or 0,
                                    ", ".join(describe_error(status or 0)))),
            ("supply voltage present", bool(voltage) and voltage > 40,
             f"{(voltage or 0) / 10.0:.1f} V"),
            ("position range is usable", (high or 0) - (low or 0) > 100,
             f"{low} .. {high}"),
            ("not overheating", (temperature or 0) < (max_temperature or 70),
             f"{temperature} C of {max_temperature} C"),
        ]

    def selftest(self, servo_id, span=100, moves=4, tolerance=10, progress=None):
        """Command a series of moves and check the servo really reached them.

        Torque is switched on, each target is commanded and read back, and the
        servo is left as it was found. This is the check to run when a servo
        looks like it is ignoring commands: it separates "the tool is not
        sending anything" from "the servo is not acting on it".

        Parameters
        ----------
        servo_id : int
            Servo id.
        span : int, optional
            How far to move either side of the starting position, in encoder
            counts. Kept small so the test is safe on an assembled robot.
        moves : int, optional
            How many moves to command.
        tolerance : int, optional
            Encoder counts of final error still counted as reaching the target.
        progress : callable or None, optional
            Called as ``progress(result)`` after each move.

        Returns
        -------
        dict
            ``checks`` from :meth:`preflight`, a ``moves`` list of per move
            dicts, and ``passed``.
        """
        checks = self.preflight(servo_id)
        low = self.read_register(servo_id, "min_position_limit")
        high = self.read_register(servo_id, "max_position_limit")
        start = self.read_register(servo_id, "present_position")
        was_on = bool(self.read_register(servo_id, "torque_enable"))

        offsets = []
        for index in range(moves):
            # Alternate either side of the start so the servo comes back.
            direction = 1 if index % 2 == 0 else -1
            offsets.append(direction * span * (1 if index < 2 else 2) // 2)
        offsets.append(0)

        results = []
        self.set_torque(servo_id, True)
        try:
            for offset in offsets:
                target = max(low + 10, min(high - 10, start + offset))
                before = self.read_register(servo_id, "present_position")
                self.set_goal_position(servo_id, target)
                began = time.monotonic()
                reached = self.wait_until_stopped(servo_id)
                result = {
                    "target": target,
                    "before": before,
                    "reached": reached,
                    "error": reached - target,
                    "seconds": time.monotonic() - began,
                    "commanded_move": abs(target - before) > tolerance,
                    "reached_target": abs(reached - target) <= tolerance,
                }
                results.append(result)
                if progress is not None:
                    progress(result)
        finally:
            self.set_goal_position(servo_id, start)
            self.wait_until_stopped(servo_id, timeout=2.0)
            if not was_on:
                self.set_torque(servo_id, False)

        asked_to_move = [r for r in results if r["commanded_move"]]
        passed = (
            all(ok for _, ok, _ in checks)
            and bool(asked_to_move)
            and all(r["reached_target"] for r in asked_to_move)
        )
        return {
            "checks": checks,
            "moves": results,
            "start": start,
            "passed": passed,
            "moved_count": sum(1 for r in asked_to_move if r["reached_target"]),
            "asked_count": len(asked_to_move),
        }

    def set_zero(self, servo_id, position=CENTER_POSITION):
        """Re-label the pose the servo is in right now.

        Writes ``homing_offset`` so that where the horn sits at this moment
        reads as ``position``. The servo does not move; only the number it
        reports changes. This is how a joint is zeroed after assembly.

        The relationship, measured on an STS3215, is
        ``present_position = (raw_encoder - homing_offset) mod 4096``. The
        wrap is not cosmetic: a servo that already carries an offset reports a
        position that has crossed the 0/4095 boundary, and the raw reading has
        to be brought back into range before a new offset can be worked out.

        Torque is switched off first and the goal is re-pointed at the new
        reading, so that re-energising the servo does not make it jump to a
        goal that was expressed in the old frame.

        Parameters
        ----------
        servo_id : int
            Servo id.
        position : int, optional
            The encoder count the current pose should report. Defaults to
            :data:`~feetech_cli.registers.CENTER_POSITION`, which is 0 degrees.

        Returns
        -------
        int
            The position the servo reports after the change.

        Raises
        ------
        FeetechError
            If the required offset falls outside the range the register can
            hold, which happens when the pose is too far from the target.
        """
        self._require_open()
        offset = self.read_register(servo_id, "homing_offset")
        present = self.read_register(servo_id, "present_position")
        raw = (present + offset) % RESOLUTION
        new_offset = raw - position
        if abs(new_offset) > MAX_HOMING_OFFSET:
            raise FeetechError(
                f"cannot zero here: the servo would need an offset of {new_offset}, and "
                f"the register only holds +/-{MAX_HOMING_OFFSET}. Move the joint closer to the "
                "target and try again."
            )
        was_on = bool(self.read_register(servo_id, "torque_enable"))
        self.set_torque(servo_id, False)
        self.write_register(servo_id, "homing_offset", new_offset)
        time.sleep(0.05)
        reported = self.read_register(servo_id, "present_position")
        # Keep the goal in the new frame so torque can be re-enabled safely.
        # This write energises the servo on its own, hence the restore below.
        self.set_goal_position(servo_id, reported)
        if not was_on:
            self.set_torque(servo_id, False)
        return reported

    def set_operating_mode(self, servo_id, mode):
        """Switch a servo between position, wheel, PWM and step mode.

        Parameters
        ----------
        servo_id : int
            Servo id.
        mode : int
            Mode number, a key of
            :data:`~feetech_cli.registers.OPERATING_MODES`.

        Returns
        -------
        int
            The mode read back from the servo.
        """
        self.set_torque(servo_id, False)
        return self.write_register(servo_id, "operating_mode", mode)


def _lookup(name):
    """Return the control table entry for a register name.

    Parameters
    ----------
    name : str
        Register name.

    Returns
    -------
    Register
        The control table entry.

    Raises
    ------
    FeetechError
        If the name is not in the control table.
    """
    if name not in CONTROL_TABLE:
        raise FeetechError(f"unknown register '{name}'")
    return CONTROL_TABLE[name]
