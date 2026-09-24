"""An in-memory FEETECH servo bus.

This backs ``feetech --simulate`` and the test suite, so that the CLI and the
interactive screen can be exercised without hardware attached. It models the
parts of the servos that the tool actually depends on: the control table, the
EEPROM lock, per servo bus speeds, the response status level, and the transmit
echo that some half duplex adapters produce.

It is a simulation and nothing more. It never stands in for a real bus
automatically; ``--simulate`` has to be asked for explicitly.
"""

from feetech_cli.protocol import INST_FACTORY_RESET
from feetech_cli.protocol import INST_PING
from feetech_cli.protocol import INST_READ
from feetech_cli.protocol import INST_SYNC_WRITE
from feetech_cli.protocol import INST_WRITE
from feetech_cli.protocol import checksum
from feetech_cli.protocol import from_sign_magnitude
from feetech_cli.registers import BAUDRATE_TABLE
from feetech_cli.registers import CONTROL_TABLE
from feetech_cli.registers import RESOLUTION

#: Addresses below this one live in EEPROM and honour the lock register.
EEPROM_END = 40

#: Factory defaults, mirroring an STS3215 straight out of the box.
DEFAULTS = {
    "model_number": 777,
    "id": 1,
    "baud_rate": 0,
    "response_status_level": 1,
    "min_position_limit": 0,
    "max_position_limit": 4095,
    "max_temperature_limit": 70,
    "max_voltage_limit": 140,
    "min_voltage_limit": 50,
    "max_torque_limit": 1000,
    "torque_limit": 1000,
    "p_coefficient": 32,
    "d_coefficient": 32,
    "cw_dead_zone": 1,
    "ccw_dead_zone": 1,
    "angular_resolution": 1,
    "lock": 1,
    "present_position": 2048,
    "goal_position": 2048,
    "present_voltage": 121,
    "present_temperature": 35,
}


class SimulatedServo:
    """A single simulated servo.

    Parameters
    ----------
    servo_id : int
        Initial servo id.
    baud_index : int, optional
        Initial value of the ``baud_rate`` register.
    response_level : int, optional
        Initial value of the ``response_status_level`` register.
    """

    def __init__(self, servo_id, baud_index=0, response_level=1):
        self.memory = bytearray(128)
        for name, value in DEFAULTS.items():
            self._store(name, value)
        self._store("id", servo_id)
        self._store("baud_rate", baud_index)
        self._store("response_status_level", response_level)
        self.error = 0
        #: True encoder reading. What the servo reports is this minus
        #: ``homing_offset``, which is how the real hardware behaves.
        self.raw_position = DEFAULTS["present_position"]
        #: Register writes that the lock rejected, for assertions in tests.
        self.rejected_writes = []

    def _store(self, name, value):
        """Write a named register directly, ignoring the lock.

        Parameters
        ----------
        name : str
            Register name.
        value : int
            Value to store.
        """
        register = CONTROL_TABLE[name]
        if register.size == 1:
            self.memory[register.address] = value & 0xFF
        else:
            self.memory[register.address] = value & 0xFF
            self.memory[register.address + 1] = (value >> 8) & 0xFF

    def get(self, name):
        """Read a named register directly.

        Parameters
        ----------
        name : str
            Register name.

        Returns
        -------
        int
            The stored value.
        """
        register = CONTROL_TABLE[name]
        if register.size == 1:
            return self.memory[register.address]
        return self.memory[register.address] | (self.memory[register.address + 1] << 8)

    @property
    def servo_id(self):
        """int: The servo's current id."""
        return self.memory[CONTROL_TABLE["id"].address]

    @property
    def baudrate(self):
        """int: The bus speed the servo currently listens at."""
        return BAUDRATE_TABLE[self.memory[CONTROL_TABLE["baud_rate"].address]]

    @property
    def locked(self):
        """bool: Whether the EEPROM is write protected."""
        return bool(self.memory[CONTROL_TABLE["lock"].address])

    @property
    def response_level(self):
        """int: The servo's ``response_status_level``."""
        return self.memory[CONTROL_TABLE["response_status_level"].address]

    def apply_write(self, address, data):
        """Apply a write instruction to the servo memory.

        EEPROM writes are dropped while the lock register is set, which is what
        the real servos do.

        Parameters
        ----------
        address : int
            First register address.
        data : sequence of int
            Bytes to store.
        """
        for offset, byte in enumerate(data):
            target = address + offset
            if target < EEPROM_END and self.locked:
                self.rejected_writes.append(target)
                continue
            self.memory[target] = byte
        goal = CONTROL_TABLE["goal_position"].address
        if address <= goal < address + len(data):
            # Measured on an STS3215: writing goal_position turns the output
            # stage on by itself. Writes to lock, homing_offset, acceleration
            # and the rest do not. A position command therefore always moves
            # the servo, whatever torque_enable said beforehand.
            self._store("torque_enable", 1)
        self._settle()

    @property
    def homing_offset(self):
        """int: The signed homing offset currently stored."""
        return from_sign_magnitude(self.get("homing_offset"), 11)

    def _settle(self):
        """Bring the reported position in line with the registers.

        Two things are modelled. A real servo accepts a ``goal_position``
        write whether or not torque is enabled, but only moves when it is
        enabled. And ``present_position`` is the raw encoder reading minus
        ``homing_offset``, so changing the offset re-labels the pose without
        the horn moving. A simulator that ignored either would teach the
        opposite of how the hardware behaves.

        Both are taken modulo :data:`~feetech_cli.registers.RESOLUTION`. The
        encoder only ever reports a value inside one turn, so an offset that
        pushes the reading past either end wraps rather than running off the
        scale. Measured on an STS3215: raw 628 with an offset of 904 reports
        3820, not -276.
        """
        offset = self.homing_offset
        if self.get("torque_enable"):
            self.raw_position = (self.get("goal_position") + offset) % RESOLUTION
        self._store(
            "present_position", (self.raw_position - offset) % RESOLUTION
        )

    def factory_reset(self):
        """Restore the factory defaults."""
        self.memory = bytearray(128)
        for name, value in DEFAULTS.items():
            self._store(name, value)
        self.raw_position = DEFAULTS["present_position"]


class SimulatedBus:
    """A serial-port lookalike carrying a set of :class:`SimulatedServo` objects.

    Only the handful of :class:`serial.Serial` members used by this package are
    implemented.

    Parameters
    ----------
    servos : sequence of SimulatedServo
        The servos hanging off the bus.
    port : str, optional
        Device name reported by the ``port`` attribute.
    baudrate : int, optional
        Initial bus speed.
    echo : bool, optional
        Echo every transmitted byte back onto the receive buffer, the way some
        half duplex adapters do.
    """

    def __init__(self, servos, port="/dev/fake", baudrate=1000000, echo=False):
        self.servos = list(servos)
        self.port = port
        self.baudrate = baudrate
        self.timeout = 0.05
        self.is_open = True
        self.echo = echo
        self.rx = bytearray()
        self.tx_log = []

    def close(self):
        """Mark the port closed."""
        self.is_open = False

    def reset_input_buffer(self):
        """Discard anything waiting to be read."""
        self.rx = bytearray()

    def flush(self):
        """No-op, writes are handled synchronously."""

    def read(self, count=1):
        """Read up to ``count`` bytes from the receive buffer.

        Parameters
        ----------
        count : int, optional
            Maximum number of bytes to return.

        Returns
        -------
        bytes
            The bytes read, shorter than ``count`` when the buffer runs dry,
            which stands in for a serial timeout.
        """
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, packet):
        """Deliver a packet to the bus and queue any answer.

        Parameters
        ----------
        packet : bytes
            A complete instruction packet.

        Returns
        -------
        int
            Number of bytes accepted.
        """
        self.tx_log.append(bytes(packet))
        if self.echo:
            self.rx.extend(packet)
        answer = self._dispatch(packet)
        if answer is not None:
            self.rx.extend(answer)
        return len(packet)

    def _dispatch(self, packet):
        """Route an instruction packet to the addressed servo.

        Parameters
        ----------
        packet : bytes
            A complete instruction packet.

        Returns
        -------
        bytes or None
            The status packet, or ``None`` when no servo answers.
        """
        if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
            return None
        servo_id, length, instruction = packet[2], packet[3], packet[4]
        params = packet[5:5 + length - 2]
        if checksum(packet[2:-1]) != packet[-1]:
            return None

        if instruction == INST_SYNC_WRITE:
            self._sync_write(params)
            return None

        for servo in self.servos:
            if servo.baudrate != self.baudrate:
                # Wrong bus speed, the servo cannot decode the packet.
                continue
            if servo.servo_id != servo_id:
                continue
            return self._handle(servo, instruction, params)
        return None

    def _sync_write(self, params):
        """Apply a broadcast sync write to every servo it addresses.

        Parameters
        ----------
        params : bytes
            Address, per servo byte count, then one id plus payload each.
        """
        if len(params) < 2:
            return
        address, data_length = params[0], params[1]
        body = params[2:]
        stride = data_length + 1
        by_id = {servo.servo_id: servo for servo in self.servos
                 if servo.baudrate == self.baudrate}
        for start in range(0, len(body) - stride + 1, stride):
            servo = by_id.get(body[start])
            if servo is not None:
                servo.apply_write(address, body[start + 1:start + stride])

    def _handle(self, servo, instruction, params):
        """Execute one instruction on one servo.

        Parameters
        ----------
        servo : SimulatedServo
            Addressed servo.
        instruction : int
            Instruction byte.
        params : bytes
            Instruction parameters.

        Returns
        -------
        bytes or None
            The status packet, or ``None`` when the servo stays silent.
        """
        if instruction == INST_PING:
            return _status(servo.servo_id, servo.error, b"")
        if instruction == INST_READ:
            address, size = params[0], params[1]
            payload = bytes(servo.memory[address:address + size])
            return _status(servo.servo_id, servo.error, payload)
        if instruction == INST_WRITE:
            servo.apply_write(params[0], params[1:])
            if servo.response_level == 0:
                return None
            return _status(servo.servo_id, servo.error, b"")
        if instruction == INST_FACTORY_RESET:
            answer = _status(servo.servo_id, servo.error, b"")
            servo.factory_reset()
            return answer
        return None


def _status(servo_id, error, params):
    """Assemble a status packet.

    Parameters
    ----------
    servo_id : int
        Answering servo id.
    error : int
        Hardware error byte.
    params : bytes
        Returned parameters.

    Returns
    -------
    bytes
        The status packet.
    """
    body = [servo_id, len(params) + 2, error] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def build_demo_bus(servo_ids=(1, 2, 3), port="/dev/simulated", baudrate=1000000):
    """Build a bus carrying a few servos, for demos and manual testing.

    Parameters
    ----------
    servo_ids : sequence of int, optional
        Ids of the servos to put on the bus.
    port : str, optional
        Device name the bus reports.
    baudrate : int, optional
        Bus speed.

    Returns
    -------
    SimulatedBus
        The simulated bus.
    """
    return SimulatedBus(
        [SimulatedServo(servo_id) for servo_id in servo_ids],
        port=port,
        baudrate=baudrate,
    )
