"""Kinds of device that can sit on a FEETECH bus, and how to tell them apart.

A FEETECH bus does not only carry servos. Any board that speaks the same
framing can share it -- a sensor or valve controller next to the joints it
works with -- but its control table is its own. Reading such a board through
the servo table produces plausible nonsense (an id of 48, a 250 kbps bus speed),
and writing through it corrupts the board while the tool reports success.

So every id can be identified before its registers are touched. This package
only knows FEETECH servos; any other device is described by a JSON *device
file* that its maker ships, and that is loaded from (in order):

1. ``--device-file PATH`` on the command line (repeatable);
2. the ``FEETECH_CLI_DEVICES`` environment variable, a list of files or
   directories separated by ``os.pathsep``;
3. every ``*.json`` in ``$XDG_CONFIG_HOME/feetech-cli/devices`` (by default
   ``~/.config/feetech-cli/devices``).

With no device file loaded, nothing is identified and every id is a servo,
exactly as before. A device file::

    {
      "name": "Example sensor board",
      "match": {"address": 0, "bytes": "34124558"},
      "settle_time": 1.0,
      "registers": {
        "model_number": {"address": 0, "size": 2},
        "id":        {"address": 7, "size": 1, "writable": true},
        "baud_rate": {"address": 8, "size": 1, "writable": true},
        "mode":      {"address": 20, "size": 1, "writable": true,
                      "values": {"0": "off", "1": "on"}},
        "pressure":  {"address": 24, "size": 2, "signed": true,
                      "scale": 0.01, "unit": "kPa", "digits": 2},
        "acc_z":     {"address": 26, "size": 2, "signed": true, "unit": "g",
                      "scale_by": {"register": "acc_range_g", "divisor": 32768}},
        "error":     {"address": 30, "size": 1, "bits": {"0": "imu", "3": "adc"}}
      },
      "info": ["id", "mode", "pressure"],
      "summary": ["mode", "pressure"]
    }

``match`` is compared against the bytes read from ``address``; the first
device file whose bytes match wins, anything else is a servo. See
:func:`profile_from_dict` for every key.
"""

import json
import os
from collections import namedtuple
from pathlib import Path

from feetech_cli.registers import BAUDRATE_TABLE
from feetech_cli.registers import CONTROL_TABLE
from feetech_cli.registers import Register

#: One kind of device on the bus.
#:
#: Attributes
#: ----------
#: name : str
#:     Human readable kind, used in messages.
#: key : str
#:     Short name to select the profile on the command line.
#: is_servo : bool
#:     Whether servo commands (torque, move, mode, ...) apply.
#: control_table : dict
#:     Register name to :class:`~feetech_cli.registers.Register`.
#: baudrate_table : dict
#:     Value of the ``baud_rate`` register to bus speed.
#: has_eeprom_lock : bool
#:     Whether EEPROM writes must be bracketed by the ``lock`` register.
#: info_registers : tuple of str or None
#:     Registers shown by ``feetech info`` without ``--all``.
#: settle_time : float
#:     Seconds the device may stay silent after its id or bus speed changes.
#: match : tuple of (int, bytes) or None
#:     Address and bytes that identify the device; None for the servo.
#: formats : dict
#:     Register name to a display rule (see :func:`format_value`).
#: summary_registers : tuple of str
#:     Registers shown on the device's ``feetech scan`` line.
#: source : str or None
#:     The device file the profile came from.
DeviceProfile = namedtuple(
    "DeviceProfile",
    [
        "name",
        "key",
        "is_servo",
        "control_table",
        "baudrate_table",
        "has_eeprom_lock",
        "info_registers",
        "settle_time",
        "match",
        "formats",
        "summary_registers",
        "source",
    ],
)

SERVO = DeviceProfile(
    name="servo",
    key="servo",
    is_servo=True,
    control_table=CONTROL_TABLE,
    baudrate_table=BAUDRATE_TABLE,
    has_eeprom_lock=True,
    info_registers=None,
    settle_time=0.05,
    match=None,
    formats={},
    summary_registers=(),
    source=None,
)

#: Environment variable listing device files or directories.
DEVICES_ENV = "FEETECH_CLI_DEVICES"

#: Named baud rate tables a device file may refer to.
BAUDRATE_TABLES = {"sts": BAUDRATE_TABLE}

_PROFILE_KEYS = {
    "name", "description", "match", "settle_time", "eeprom_lock", "baudrate_table",
    "registers", "info", "summary",
}
_REGISTER_KEYS = {
    "address", "size", "writable", "signed", "area", "description",
    "values", "bits", "scale", "scale_by", "unit", "digits",
}


class DeviceFileError(ValueError):
    """A device file that cannot be used, with the file and key named."""


def _require(condition, source, message):
    """Raise :class:`DeviceFileError` unless ``condition`` holds."""
    if not condition:
        raise DeviceFileError(f"{source}: {message}")


def _int_keyed(mapping, source, where):
    """Turn a JSON object with integer-string keys into ``{int: str}``."""
    _require(isinstance(mapping, dict), source, f"{where} must be an object")
    out = {}
    for key, label in mapping.items():
        try:
            number = int(key, 0)
        except ValueError:
            raise DeviceFileError(f"{source}: {where} key '{key}' is not an integer") from None
        out[number] = str(label)
    return out


def _register_from_dict(name, spec, source):
    """Build one control table entry and its display rule."""
    where = f"register '{name}'"
    _require(isinstance(spec, dict), source, f"{where} must be an object")
    unknown = set(spec) - _REGISTER_KEYS
    _require(not unknown, source, f"{where} has unknown key(s) {sorted(unknown)}")
    address = spec.get("address")
    size = spec.get("size", 1)
    _require(
        isinstance(address, int) and 0 <= address <= 255, source,
        f"{where} needs an integer 'address' 0-255",
    )
    _require(size in (1, 2), source, f"{where} 'size' must be 1 or 2")
    register = Register(
        address,
        size,
        str(spec.get("area", "sram")),
        bool(spec.get("writable", False)),
        None,
        bool(spec.get("signed", False)),
    )
    rule = {}
    if "values" in spec:
        rule["values"] = _int_keyed(spec["values"], source, f"{where} 'values'")
    if "bits" in spec:
        bits = _int_keyed(spec["bits"], source, f"{where} 'bits'")
        _require(
            all(0 <= bit < 8 * size for bit in bits), source,
            f"{where} 'bits' must be bit numbers below {8 * size}",
        )
        rule["bits"] = bits
    if "scale" in spec:
        _require(
            isinstance(spec["scale"], (int, float)), source, f"{where} 'scale' must be a number"
        )
        rule["scale"] = float(spec["scale"])
    if "scale_by" in spec:
        by = spec["scale_by"]
        _require(
            isinstance(by, dict)
            and isinstance(by.get("register"), str)
            and isinstance(by.get("divisor"), (int, float))
            and by["divisor"] != 0,
            source,
            f"{where} 'scale_by' needs 'register' and a non-zero 'divisor'",
        )
        rule["scale_by"] = (by["register"], float(by["divisor"]))
    if "unit" in spec:
        rule["unit"] = str(spec["unit"])
    if "digits" in spec:
        _require(
            isinstance(spec["digits"], int) and spec["digits"] >= 0, source,
            f"{where} 'digits' must be a non-negative integer",
        )
        rule["digits"] = spec["digits"]
    return register, rule


def profile_from_dict(data, source="<device file>", key=None):
    """Build a :class:`DeviceProfile` from the contents of a device file.

    Parameters
    ----------
    data : dict
        The parsed JSON. Keys:

        - ``name`` (str, required): what the device is called in messages.
        - ``match`` (object, required): ``address`` (int) and ``bytes`` (hex
          string) found there on this kind of device and no other.
        - ``registers`` (object, required): register name to ``address``,
          ``size`` (1 or 2, default 1), ``writable`` (default false),
          ``signed`` (two's complement, default false), ``area`` (default
          ``"sram"``), and optionally how to show it: ``values`` (number to
          label), ``bits`` (bit number to label), ``scale`` or ``scale_by``
          (``{"register": name, "divisor": n}``: value * that register / n),
          ``unit`` and ``digits``.
        - ``settle_time`` (seconds, default 1.0): how long the device may be
          silent after its id or bus speed changes.
        - ``eeprom_lock`` (default false): whether EEPROM writes need the
          servo-style ``lock`` register.
        - ``baudrate_table`` (default ``"sts"``): ``"sts"`` or an object of
          ``baud_rate`` value to bits per second.
        - ``info`` / ``summary``: registers ``feetech info`` and the scan line
          show.
        - ``description``: free text, ignored.
    source : str, optional
        Where the data came from, for error messages.
    key : str or None, optional
        Short name for the command line; derived from ``name`` when omitted.

    Returns
    -------
    DeviceProfile
        The profile.

    Raises
    ------
    DeviceFileError
        If a key is missing, misspelt or of the wrong type.
    """
    _require(isinstance(data, dict), source, "must contain a JSON object")
    unknown = set(data) - _PROFILE_KEYS
    _require(not unknown, source, f"unknown key(s) {sorted(unknown)}")
    name = data.get("name")
    _require(isinstance(name, str) and name.strip(), source, "needs a 'name'")

    match = data.get("match")
    _require(
        isinstance(match, dict), source, "needs a 'match' object with 'address' and 'bytes'"
    )
    try:
        match_bytes = bytes.fromhex(str(match.get("bytes", "")))
    except ValueError:
        raise DeviceFileError(f"{source}: 'match.bytes' must be a hex string") from None
    match_address = match.get("address", 0)
    _require(
        isinstance(match_address, int) and match_address >= 0 and match_bytes, source,
        "'match' needs a non-negative 'address' and at least one byte",
    )

    registers = data.get("registers")
    _require(
        isinstance(registers, dict) and registers, source, "needs a non-empty 'registers' object"
    )
    table = {}
    formats = {}
    for reg_name, spec in registers.items():
        table[reg_name], rule = _register_from_dict(reg_name, spec, source)
        if rule:
            formats[reg_name] = rule
    for reg_name, rule in formats.items():
        if "scale_by" in rule:
            _require(
                rule["scale_by"][0] in table, source,
                f"register '{reg_name}' scales by unknown register '{rule['scale_by'][0]}'",
            )

    baud = data.get("baudrate_table", "sts")
    if isinstance(baud, str):
        _require(
            baud in BAUDRATE_TABLES, source,
            f"unknown 'baudrate_table' '{baud}' (known: {sorted(BAUDRATE_TABLES)})",
        )
        baudrate_table = BAUDRATE_TABLES[baud]
    else:
        baudrate_table = {
            index: int(rate)
            for index, rate in _int_keyed(baud, source, "'baudrate_table'").items()
        }

    lists = {}
    for list_key in ("info", "summary"):
        names = data.get(list_key, [])
        _require(
            isinstance(names, list), source, f"'{list_key}' must be a list of register names"
        )
        missing = [n for n in names if n not in table]
        _require(not missing, source, f"'{list_key}' names unknown register(s) {missing}")
        lists[list_key] = tuple(names)

    settle_time = data.get("settle_time", 1.0)
    _require(
        isinstance(settle_time, (int, float)) and settle_time >= 0, source,
        "'settle_time' must be a non-negative number of seconds",
    )

    return DeviceProfile(
        name=name.strip(),
        key=key or "-".join(name.lower().split()),
        is_servo=False,
        control_table=table,
        baudrate_table=baudrate_table,
        has_eeprom_lock=bool(data.get("eeprom_lock", False)),
        info_registers=lists["info"] or None,
        settle_time=float(settle_time),
        match=(match_address, match_bytes),
        formats=formats,
        summary_registers=lists["summary"],
        source=source,
    )


def load_profile(path):
    """Load one device file.

    Parameters
    ----------
    path : str or os.PathLike
        A ``.json`` device file.

    Returns
    -------
    DeviceProfile
        The profile; its ``key`` is the file name without ``.json``.

    Raises
    ------
    DeviceFileError
        If the file cannot be read or is not a valid device file.
    """
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise DeviceFileError(f"{path}: cannot read ({error.strerror})") from None
    except json.JSONDecodeError as error:
        raise DeviceFileError(f"{path}: not valid JSON ({error})") from None
    return profile_from_dict(data, source=str(path), key=path.stem)


def default_device_dir(environ=None):
    """Return the per-user directory searched for device files.

    Parameters
    ----------
    environ : mapping or None, optional
        Environment to read ``XDG_CONFIG_HOME`` from; ``os.environ`` if None.

    Returns
    -------
    pathlib.Path
        ``$XDG_CONFIG_HOME/feetech-cli/devices`` or
        ``~/.config/feetech-cli/devices``.
    """
    environ = os.environ if environ is None else environ
    base = environ.get("XDG_CONFIG_HOME") or str(Path.home() / ".config")
    return Path(base) / "feetech-cli" / "devices"


def _expand(entry):
    """List the device files an entry names: itself, or a directory's *.json."""
    path = Path(entry).expanduser()
    if path.is_dir():
        return sorted(path.glob("*.json"))
    return [path]


def load_profiles(paths=(), environ=None, search_default=True):
    """Load every device file that applies, in priority order.

    Parameters
    ----------
    paths : sequence of str, optional
        Files or directories given on the command line; searched first.
    environ : mapping or None, optional
        Environment; ``os.environ`` if None.
    search_default : bool, optional
        Whether to search :func:`default_device_dir` too.

    Returns
    -------
    list of DeviceProfile
        Every profile, earlier sources first. A key seen twice keeps its first
        definition, so a file on the command line overrides an installed one.

    Raises
    ------
    DeviceFileError
        If a named file is missing or any file is invalid. A broken device file
        is reported, never skipped: skipping it would identify that device as a
        servo and write servo registers into it.
    """
    environ = os.environ if environ is None else environ
    entries = list(paths)
    env_value = environ.get(DEVICES_ENV, "")
    entries += [part for part in env_value.split(os.pathsep) if part]
    if search_default:
        default_dir = default_device_dir(environ)
        if default_dir.is_dir():
            entries.append(default_dir)

    profiles = []
    seen = set()
    for entry in entries:
        for path in _expand(entry):
            profile = load_profile(path)
            if profile.key in seen:
                continue
            seen.add(profile.key)
            profiles.append(profile)
    return profiles


def identity_span(profiles):
    """Bytes to read from address 0 to test every profile's ``match``.

    Parameters
    ----------
    profiles : sequence of DeviceProfile
        Loaded device profiles.

    Returns
    -------
    int
        0 when there is nothing to identify.
    """
    ends = [p.match[0] + len(p.match[1]) for p in profiles if p.match is not None]
    return max(ends, default=0)


def identify(identity, profiles):
    """Tell which kind of device produced an identity read.

    Parameters
    ----------
    identity : bytes
        Bytes read from address 0, :func:`identity_span` of them.
    profiles : sequence of DeviceProfile
        Loaded device profiles, in priority order.

    Returns
    -------
    DeviceProfile
        The first profile whose ``match`` bytes are found, else :data:`SERVO`.
    """
    identity = bytes(identity)
    for profile in profiles:
        if profile.match is None:
            continue
        address, expected = profile.match
        if identity[address:address + len(expected)] == expected:
            return profile
    return SERVO


def format_value(profile, name, value, context=None):
    """Render a register value of a device-file profile.

    Parameters
    ----------
    profile : DeviceProfile
        The device's profile.
    name : str
        Register name.
    value : int
        Raw register value.
    context : dict or None, optional
        Other register values of the same device, for ``scale_by``.

    Returns
    -------
    str or None
        The rendering, or None when the device file says nothing about it
        (or a ``scale_by`` reference was not read).
    """
    rule = profile.formats.get(name)
    if not rule:
        return None
    if "values" in rule:
        return "{} ({})".format(value, rule["values"].get(value, "unknown"))
    if "bits" in rule:
        width = 2 * profile.control_table[name].size
        flags = [label for bit, label in sorted(rule["bits"].items()) if value & (1 << bit)]
        return "0x{:0{width}X} ({})".format(
            value, ", ".join(flags) if flags else "none", width=width
        )
    scale = rule.get("scale")
    if "scale_by" in rule:
        by_register, divisor = rule["scale_by"]
        reference = (context or {}).get(by_register)
        if reference is None:
            return None
        scale = reference / divisor
    unit = rule.get("unit", "")
    if scale is None:
        return f"{value} ({unit})" if unit else None
    digits = rule.get("digits", 3)
    sign = "+" if profile.control_table[name].twos_complement else ""
    text = f"{value * scale:{sign}.{digits}f}"
    return f"{value} ({text} {unit})" if unit else f"{value} ({text})"
