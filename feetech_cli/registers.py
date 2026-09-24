"""Control table of the FEETECH STS/SMS serial bus servos.

Register addresses follow the STS/SMS memory table used by the STS3215 and
its siblings. The baud rate index table is the one implemented by FEETECH's
own tooling; note that some third party tables (notably LeRobot's) list
indices 5-7 as 57600/38400/19200 instead, which does not match the servos.

Sources
-------
- https://github.com/commanderfun/STS3215/blob/main/REGISTER_REFERENCE.md
- https://github.com/hello-robot/stretch4_body/blob/master/stretch4_body/core/feetech/feetech_SM_servo.py
- https://github.com/AkariGroup/feetech_setup/blob/main/set_baudrate.py
- https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py
"""

from collections import namedtuple

#: One entry of the control table.
#:
#: Attributes
#: ----------
#: address : int
#:     Register address on the servo.
#: size : int
#:     Register width in bytes.
#: area : str
#:     ``"eeprom"`` for persistent registers, ``"sram"`` for volatile ones.
#: writable : bool
#:     Whether the register accepts writes.
#: sign_bit : int or None
#:     Index of the sign bit for sign-magnitude encoded registers.
Register = namedtuple("Register", ["address", "size", "area", "writable", "sign_bit"])


def _eeprom(address, size, writable=True, sign_bit=None):
    """Build an EEPROM register entry.

    Parameters
    ----------
    address : int
        Register address.
    size : int
        Register width in bytes.
    writable : bool, optional
        Whether the register accepts writes.
    sign_bit : int or None, optional
        Sign bit index for sign-magnitude registers.

    Returns
    -------
    Register
        The register entry.
    """
    return Register(address, size, "eeprom", writable, sign_bit)


def _sram(address, size, writable=True, sign_bit=None):
    """Build an SRAM register entry.

    Parameters
    ----------
    address : int
        Register address.
    size : int
        Register width in bytes.
    writable : bool, optional
        Whether the register accepts writes.
    sign_bit : int or None, optional
        Sign bit index for sign-magnitude registers.

    Returns
    -------
    Register
        The register entry.
    """
    return Register(address, size, "sram", writable, sign_bit)


CONTROL_TABLE = {
    # --- EEPROM -----------------------------------------------------------
    "firmware_major": _eeprom(0, 1, writable=False),
    "firmware_minor": _eeprom(1, 1, writable=False),
    "model_number": _eeprom(3, 2, writable=False),
    "id": _eeprom(5, 1),
    "baud_rate": _eeprom(6, 1),
    "return_delay": _eeprom(7, 1),
    "response_status_level": _eeprom(8, 1),
    "min_position_limit": _eeprom(9, 2),
    "max_position_limit": _eeprom(11, 2),
    "max_temperature_limit": _eeprom(13, 1),
    "max_voltage_limit": _eeprom(14, 1),
    "min_voltage_limit": _eeprom(15, 1),
    "max_torque_limit": _eeprom(16, 2),
    "phase": _eeprom(18, 1),
    "unloading_condition": _eeprom(19, 1),
    "led_alarm_condition": _eeprom(20, 1),
    "p_coefficient": _eeprom(21, 1),
    "d_coefficient": _eeprom(22, 1),
    "i_coefficient": _eeprom(23, 1),
    "minimum_startup_force": _eeprom(24, 2),
    "cw_dead_zone": _eeprom(26, 1),
    "ccw_dead_zone": _eeprom(27, 1),
    "protection_current": _eeprom(28, 2),
    "angular_resolution": _eeprom(30, 1),
    "homing_offset": _eeprom(31, 2, sign_bit=11),
    "operating_mode": _eeprom(33, 1),
    "protective_torque": _eeprom(34, 1),
    "protection_time": _eeprom(35, 1),
    "overload_torque": _eeprom(36, 1),
    "velocity_p_coefficient": _eeprom(37, 1),
    "over_current_protection_time": _eeprom(38, 1),
    "velocity_i_coefficient": _eeprom(39, 1),
    # --- SRAM -------------------------------------------------------------
    "torque_enable": _sram(40, 1),
    "acceleration": _sram(41, 1),
    "goal_position": _sram(42, 2, sign_bit=15),
    "goal_time": _sram(44, 2),
    "goal_velocity": _sram(46, 2, sign_bit=15),
    "torque_limit": _sram(48, 2),
    "lock": _sram(55, 1),
    "present_position": _sram(56, 2, writable=False, sign_bit=15),
    "present_velocity": _sram(58, 2, writable=False, sign_bit=15),
    "present_load": _sram(60, 2, writable=False, sign_bit=10),
    "present_voltage": _sram(62, 1, writable=False),
    "present_temperature": _sram(63, 1, writable=False),
    "async_write_flag": _sram(64, 1, writable=False),
    "status": _sram(65, 1, writable=False),
    "moving": _sram(66, 1, writable=False),
    "present_current": _sram(69, 2, writable=False),
}

#: Register value 0-7 written to ``baud_rate`` and the bus speed it selects.
BAUDRATE_TABLE = {
    0: 1000000,
    1: 500000,
    2: 250000,
    3: 128000,
    4: 115200,
    5: 76800,
    6: 57600,
    7: 38400,
}

#: Inverse of :data:`BAUDRATE_TABLE`.
BAUDRATE_INDEX = {baud: index for index, baud in BAUDRATE_TABLE.items()}

#: Bus speeds tried when auto detecting, fastest and most common first.
SCAN_BAUDRATES = [1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400]

#: Value written to the ``lock`` register to allow EEPROM writes.
EEPROM_UNLOCKED = 0

#: Value written to the ``lock`` register to write protect the EEPROM.
EEPROM_LOCKED = 1

#: ``operating_mode`` values.
OPERATING_MODES = {
    0: "position",
    1: "wheel",
    2: "pwm",
    3: "step",
}

#: Encoder counts per full turn for the STS/SMS series.
RESOLUTION = 4096

#: Encoder count that corresponds to the middle of the travel range.
CENTER_POSITION = RESOLUTION // 2

#: Largest magnitude the sign-magnitude ``homing_offset`` register can hold.
#: The sign lives in bit 11, so 11 bits are left for the value.
MAX_HOMING_OFFSET = (1 << 11) - 1

#: Highest servo id scanned by default. FEETECH servos accept ids up to 253,
#: but a typical arm or hand uses only the first handful.
DEFAULT_MAX_ID = 16

#: Lowest servo id scanned by default. Id 0 is valid but unusual, and 254 is
#: the broadcast address.
DEFAULT_MIN_ID = 1

#: Registers shown by the ``info`` command, in display order.
INFO_REGISTERS = [
    "id",
    "model_number",
    "firmware_major",
    "firmware_minor",
    "baud_rate",
    "operating_mode",
    "response_status_level",
    "min_position_limit",
    "max_position_limit",
    "homing_offset",
    "max_torque_limit",
    "max_temperature_limit",
    "min_voltage_limit",
    "max_voltage_limit",
    "p_coefficient",
    "i_coefficient",
    "d_coefficient",
    "torque_enable",
    "acceleration",
    "goal_position",
    "goal_velocity",
    "torque_limit",
    "lock",
    "present_position",
    "present_velocity",
    "present_load",
    "present_voltage",
    "present_temperature",
    "present_current",
    "moving",
    "status",
]


def position_to_degree(position):
    """Convert an encoder count into degrees around the centre position.

    Parameters
    ----------
    position : int
        Encoder count.

    Returns
    -------
    float
        Angle in degrees, 0.0 at :data:`CENTER_POSITION`.
    """
    return (position - CENTER_POSITION) * 360.0 / RESOLUTION


def degree_to_position(degree):
    """Convert an angle in degrees into an encoder count.

    Parameters
    ----------
    degree : float
        Angle in degrees, 0.0 meaning :data:`CENTER_POSITION`.

    Returns
    -------
    int
        Encoder count.
    """
    return int(round(degree * RESOLUTION / 360.0)) + CENTER_POSITION


def format_baud(baud):
    """Render a bus speed in a compact human readable form.

    Parameters
    ----------
    baud : int
        Bus speed in bits per second.

    Returns
    -------
    str
        For example ``"1.00M"`` or ``"115k"``.
    """
    if baud >= 1000000:
        return f"{baud / 1000000:.2f}M"
    if baud >= 1000:
        return f"{baud / 1000:.0f}k"
    return str(baud)
