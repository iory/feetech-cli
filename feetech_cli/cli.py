"""Command line interface for configuring FEETECH serial bus servos."""

import argparse
import sys

import colorama
from colorama import Fore
from colorama import Style

from feetech_cli.controller import FeetechServoController
from feetech_cli.controller import list_candidate_ports
from feetech_cli.protocol import FeetechError
from feetech_cli.registers import BAUDRATE_INDEX
from feetech_cli.registers import BAUDRATE_TABLE
from feetech_cli.registers import CENTER_POSITION
from feetech_cli.registers import CONTROL_TABLE
from feetech_cli.registers import DEFAULT_MAX_ID
from feetech_cli.registers import DEFAULT_MIN_ID
from feetech_cli.registers import INFO_REGISTERS
from feetech_cli.registers import OPERATING_MODES
from feetech_cli.registers import degree_to_position
from feetech_cli.registers import format_baud
from feetech_cli.registers import position_to_degree
from feetech_cli.tui import ServoTui


def build_parser():
    """Build the argument parser.

    Returns
    -------
    argparse.ArgumentParser
        The configured parser.
    """
    parser = argparse.ArgumentParser(
        prog="feetech",
        description=(
            "Inspect and configure FEETECH STS/SMS/SCS serial bus servos. "
            "With no sub command an interactive status screen is shown."
        ),
    )
    parser.add_argument(
        "-p", "--port", default=None,
        help="serial device, e.g. /dev/ttyUSB0. Auto detected when omitted.",
    )
    parser.add_argument(
        "-b", "--baudrate", type=int, default=None,
        help=(
            "bus speed. Auto detected when omitted by probing {}.".format(
                ", ".join(str(rate) for rate in sorted(BAUDRATE_INDEX, reverse=True))
            )
        ),
    )
    parser.add_argument(
        "--min-id", type=int, default=DEFAULT_MIN_ID,
        help="lowest servo id probed when scanning the bus (default: %(default)s)",
    )
    parser.add_argument(
        "--max-id", type=int, default=DEFAULT_MAX_ID,
        help="highest servo id probed when scanning the bus (default: %(default)s)",
    )
    parser.add_argument(
        "--timeout", type=float, default=0.05,
        help="serial read timeout in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--scs", action="store_true",
        help="talk to the older big endian SCS series instead of STS/SMS",
    )
    parser.add_argument(
        "--simulate", action="store_true",
        help=(
            "run against a simulated bus instead of hardware, to try the "
            "interface out. Nothing is sent to a real servo."
        ),
    )

    sub = parser.add_subparsers(dest="command")

    sub.add_parser("ports", help="list serial ports that could carry a servo bus")

    scan = sub.add_parser("scan", help="find the servos on the bus")
    scan.add_argument(
        "--all-baudrates", action="store_true",
        help="probe every supported bus speed, not just the connected one",
    )

    info = sub.add_parser("info", help="dump the control table of one servo")
    info.add_argument("id", type=int, nargs="?", help="servo id (default: the only one found)")
    info.add_argument("--all", action="store_true", help="show every register")

    set_id = sub.add_parser("set-id", help="change the id of a servo")
    set_id.add_argument("old_id", type=int, help="current servo id")
    set_id.add_argument("new_id", type=int, help="new servo id, 0-253")
    set_id.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation")

    set_baud = sub.add_parser("set-baud", help="change the bus speed a servo uses")
    set_baud.add_argument("id", type=int, help="servo id")
    # Stored under a distinct name: a positional called "baudrate" would land
    # on the same namespace attribute as the global --baudrate option and
    # silently make the CLI open the port at the speed it is about to set.
    set_baud.add_argument(
        "new_baudrate", metavar="baudrate", type=int,
        help="new bus speed, one of {}".format(
            ", ".join(str(rate) for rate in sorted(BAUDRATE_INDEX))
        ),
    )
    set_baud.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation")

    read = sub.add_parser("read", help="read one register")
    read.add_argument("id", type=int, help="servo id")
    read.add_argument("register", help="register name, see 'feetech registers'")

    write = sub.add_parser("write", help="write one register")
    write.add_argument("id", type=int, help="servo id")
    write.add_argument("register", help="register name, see 'feetech registers'")
    write.add_argument("value", type=int, help="value to write")

    sub.add_parser("registers", help="list the register names of the control table")

    move = sub.add_parser("move", help="command a servo to a position")
    move.add_argument("id", type=int, help="servo id")
    target = move.add_mutually_exclusive_group(required=True)
    target.add_argument("--position", type=int, help="target encoder count, 0-4095")
    target.add_argument("--degree", type=float, help="target angle in degrees, 0 is centre")
    move.add_argument("--speed", type=int, default=None, help="travel speed")
    move.add_argument("--acceleration", type=int, default=None, help="acceleration ramp")

    torque = sub.add_parser("torque", help="enable or disable the output stage")
    torque.add_argument("id", type=int, help="servo id")
    torque.add_argument("state", choices=["on", "off"], help="desired state")

    mode = sub.add_parser("mode", help="change the operating mode")
    mode.add_argument("id", type=int, help="servo id")
    mode.add_argument(
        "mode", choices=sorted(OPERATING_MODES.values()), help="operating mode",
    )

    reset = sub.add_parser(
        "factory-reset", help="restore a servo to its factory defaults",
    )
    reset.add_argument("id", type=int, help="servo id")
    reset.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation")

    set_zero = sub.add_parser(
        "set-zero",
        help="call the pose the servo is in right now zero degrees",
    )
    set_zero.add_argument(
        "id", type=int, nargs="?", help="servo id (default: the only one found)"
    )
    target = set_zero.add_mutually_exclusive_group()
    target.add_argument(
        "--degree", type=float, default=None,
        help="call the current pose this angle instead of 0",
    )
    target.add_argument(
        "--position", type=int, default=None,
        help="call the current pose this encoder count instead of 2048",
    )
    set_zero.add_argument(
        "-y", "--yes", action="store_true", help="do not ask for confirmation"
    )

    selftest = sub.add_parser(
        "selftest",
        help="command a series of moves and check the servo really made them",
    )
    selftest.add_argument(
        "id", type=int, nargs="?", help="servo id (default: the only one found)"
    )
    selftest.add_argument(
        "--span", type=int, default=100,
        help="how far to move either side of the start, in encoder counts "
             "(default: %(default)s, about 9 degrees)",
    )
    selftest.add_argument(
        "--moves", type=int, default=4,
        help="how many moves to command (default: %(default)s)",
    )

    sub.add_parser("monitor", help="interactive status screen (the default)")

    return parser


def confirm(question, assume_yes):
    """Ask the user to confirm a destructive action.

    Parameters
    ----------
    question : str
        Question to show.
    assume_yes : bool
        When ``True`` the question is skipped and the action is approved.

    Returns
    -------
    bool
        ``True`` when the action should go ahead.
    """
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        print(
            f"{Fore.RED}Refusing to continue without confirmation. Pass -y when running "
            f"non-interactively.{Style.RESET_ALL}"
        )
        return False
    answer = input(f"{question} [y/N] ").strip().lower()
    return answer in ("y", "yes")


def make_controller(args):
    """Build a controller from the parsed global options.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.

    Returns
    -------
    FeetechServoController
        A controller that has not been opened yet.
    """
    return FeetechServoController(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        little_endian=not args.scs,
        min_id=args.min_id,
        max_id=args.max_id,
    )


def attach_simulator(controller, args):
    """Point a controller at a simulated bus instead of a serial port.

    Only ``--simulate`` reaches this. A missing adapter or an unresponsive
    servo never falls back to the simulator: a bus that is not there has to
    report itself as not there.

    Parameters
    ----------
    controller : FeetechServoController
        A controller that has not been opened yet.
    args : argparse.Namespace
        Parsed arguments.
    """
    from feetech_cli.protocol import PacketHandler
    from feetech_cli.simulator import build_demo_bus

    baudrate = args.baudrate or 1000000
    bus = build_demo_bus(servo_ids=(1, 2, 3), baudrate=baudrate)
    controller.requested_port = bus.port
    controller.requested_baudrate = baudrate

    def open_port(device, rate):
        """Hand the controller the simulated bus."""
        bus.port = device
        bus.baudrate = rate
        bus.is_open = True
        bus.rx = bytearray()
        controller.serial = bus
        controller.baudrate = rate
        controller.packet_handler = PacketHandler(
            bus, little_endian=controller.little_endian
        )
        controller.silent_servos = set()

    controller._open_port = open_port
    controller.is_simulated = True
    print(
        f"{Fore.YELLOW}Simulated bus: three servos at ids 1, 2 and 3. "
        f"No hardware is involved.{Style.RESET_ALL}"
    )


def command_ports(args):
    """Print the serial ports that could carry a servo bus.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.

    Returns
    -------
    int
        Process exit status.
    """
    ports = list_candidate_ports()
    if not ports:
        print("No USB serial adapter found.")
        return 1
    for port in ports:
        print(
            f"{port.device}  {port.description}  vid=0x{port.vid:04X} pid=0x{port.pid:04X}"
        )
    return 0


def command_scan(args, controller):
    """Print the servos found on the bus.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    if args.all_baudrates:
        results = controller.scan_all_baudrates(
            progress=lambda baud, found: print(
                "  {:>9}bps: {}".format(
                    format_baud(baud),
                    " ".join(str(sid) for sid in found) if found else "-",
                )
            )
        )
        if not results:
            print(f"{Fore.RED}No servo answered at any speed.{Style.RESET_ALL}")
            return 1
        return 0

    found = controller.scan()
    if not found:
        _print_empty_bus(controller)
        return 1
    print(
        f"Found {len(found)} servo(s) on {controller.serial.port} "
        f"at {format_baud(controller.baudrate)}bps:"
    )
    for servo_id in found:
        model = controller.read_register(servo_id, "model_number")
        position = controller.read_register(servo_id, "present_position")
        voltage = controller.read_register(servo_id, "present_voltage")
        temperature = controller.read_register(servo_id, "present_temperature")
        degree = position_to_degree(position)
        print(
            f"  id {servo_id:>3}  model {model:>5}  "
            f"position {position:>5} ({degree:+.1f} deg)  "
            f"{voltage / 10.0:.1f}V  {temperature}C"
        )
    return 0


def resolve_servo_id(controller, servo_id):
    """Pick the servo to act on when the user did not name one.

    Parameters
    ----------
    controller : FeetechServoController
        An open controller.
    servo_id : int or None
        Id given on the command line, or ``None``.

    Returns
    -------
    int
        The servo id to use.

    Raises
    ------
    FeetechError
        If no id was given and the bus does not hold exactly one servo.
    """
    if servo_id is not None:
        return servo_id
    found = controller.scan()
    if len(found) == 1:
        return found[0]
    if not found:
        raise FeetechError("no servo answered on the bus")
    raise FeetechError(
        "several servos answered ({}), name the one to use".format(
            ", ".join(str(sid) for sid in found)
        )
    )


def command_info(args, controller):
    """Print the control table of one servo.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    servo_id = resolve_servo_id(controller, args.id)
    names = list(CONTROL_TABLE) if args.all else INFO_REGISTERS
    values = controller.dump(servo_id, names)
    width = max(len(name) for name in names)
    print(f"Servo {servo_id} on {controller.serial.port} at {format_baud(controller.baudrate)}bps")
    for name in names:
        register = CONTROL_TABLE[name]
        value = values[name]
        print(
            "  {:<{width}}  addr {:>3}  {}".format(
                name, register.address, _annotate(name, value), width=width
            )
        )
    return 0


def _annotate(name, value):
    """Render a register value with a unit or label where one applies.

    Parameters
    ----------
    name : str
        Register name.
    value : int or None
        Register value.

    Returns
    -------
    str
        The rendered value.
    """
    if value is None:
        return "no data"
    if name == "baud_rate":
        baud = BAUDRATE_TABLE.get(value)
        if baud is None:
            return f"{value} (unknown index)"
        return f"{value} ({format_baud(baud)}bps)"
    if name == "operating_mode":
        return "{} ({})".format(value, OPERATING_MODES.get(value, "unknown"))
    if name in ("present_position", "goal_position"):
        return f"{value} ({position_to_degree(value):+.1f} deg)"
    if name == "present_voltage":
        return f"{value} ({value / 10.0:.1f} V)"
    if name == "present_temperature":
        return f"{value} (C)"
    if name == "present_load":
        return f"{value} ({value / 10.0:+.1f} %)"
    if name == "present_current":
        return f"{value} ({value * 6.5:.0f} mA)"
    if name == "torque_enable":
        return "{} ({})".format(value, "on" if value else "off")
    if name == "lock":
        return "{} ({})".format(value, "locked" if value else "unlocked")
    return str(value)


def command_set_id(args, controller):
    """Change the id of a servo.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    if not confirm(
        f"Change servo {args.old_id} to id {args.new_id}? This writes the servo EEPROM.",
        args.yes,
    ):
        print("Cancelled.")
        return 1
    confirmed = controller.set_id(args.old_id, args.new_id)
    print(
        f"{Fore.GREEN}Servo {args.old_id} is now id {confirmed}.{Style.RESET_ALL}"
    )
    return 0


def command_set_baud(args, controller):
    """Change the bus speed a servo uses.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    if not confirm(
        f"Set servo {args.id} to {format_baud(args.new_baudrate)}bps? "
        f"This writes the servo EEPROM.",
        args.yes,
    ):
        print("Cancelled.")
        return 1
    confirmed = controller.set_baudrate(args.id, args.new_baudrate)
    print(
        f"{Fore.GREEN}Servo {args.id} now runs at {format_baud(confirmed)}bps.{Style.RESET_ALL}"
    )
    print(f"Pass --baudrate {confirmed} on later calls, or let it be auto detected.")
    return 0


def command_read(args, controller):
    """Read one register and print it.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    value = controller.read_register(args.id, args.register)
    print(f"{args.register} = {_annotate(args.register, value)}")
    return 0


def command_write(args, controller):
    """Write one register and print the value read back.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    if args.register in ("id", "baud_rate"):
        print(
            "{}Use 'feetech set-{}' instead, it reconnects and verifies the "
            "change.{}".format(
                Fore.YELLOW, "id" if args.register == "id" else "baud", Style.RESET_ALL
            )
        )
        return 1
    value = controller.write_register(args.id, args.register, args.value)
    print(f"{args.register} = {_annotate(args.register, value)}")
    if value != args.value:
        print(
            f"{Fore.RED}Read back {value} but {args.value} was written; "
            f"the servo rejected or clamped the value.{Style.RESET_ALL}"
        )
        return 1
    return 0


def command_registers(args):
    """List the control table.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.

    Returns
    -------
    int
        Process exit status.
    """
    width = max(len(name) for name in CONTROL_TABLE)
    for name, register in sorted(CONTROL_TABLE.items(), key=lambda kv: kv[1].address):
        print(
            "  {:<{width}}  addr {:>3}  {} byte  {:<6}  {}".format(
                name, register.address, register.size, register.area,
                "rw" if register.writable else "r",
                width=width,
            )
        )
    return 0


def command_move(args, controller):
    """Command a servo to a position.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    if args.degree is not None:
        position = degree_to_position(args.degree)
    else:
        position = args.position
    if args.acceleration is not None:
        controller.set_acceleration(args.id, args.acceleration)
    if args.speed is not None:
        controller.set_goal_velocity(args.id, args.speed)
    controller.set_torque(args.id, True)
    controller.set_goal_position(args.id, position)
    print(
        f"Servo {args.id} commanded to {position} ({position_to_degree(position):+.1f} deg)."
    )
    return 0


def command_torque(args, controller):
    """Enable or disable the output stage of a servo.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    state = args.state == "on"
    value = controller.set_torque(args.id, state)
    print("Servo {} torque is {}.".format(args.id, "on" if value else "off"))
    return 0


def command_mode(args, controller):
    """Change the operating mode of a servo.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    wanted = {name: number for number, name in OPERATING_MODES.items()}[args.mode]
    value = controller.set_operating_mode(args.id, wanted)
    print(
        "Servo {} operating mode is {} ({}).".format(
            args.id, OPERATING_MODES.get(value, "unknown"), value
        )
    )
    return 0


def command_factory_reset(args, controller):
    """Restore a servo to its factory defaults.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    if not confirm(
        f"Factory reset servo {args.id}? Its id, calibration and limits are lost and "
        "it returns to id 1 at 1000000bps.",
        args.yes,
    ):
        print("Cancelled.")
        return 1
    controller.factory_reset(args.id)
    print("Reset sent. Power cycle the servo, then run 'feetech scan'.")
    return 0


def command_monitor(args, controller):
    """Run the interactive status screen.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    found = controller.scan()
    if not found:
        # Start anyway and wait. A servo plugged in later
        # is picked up without restarting the tool.
        print(
            f"{Fore.YELLOW}No servo answered yet. Waiting for one to appear."
            f"{Style.RESET_ALL}"
        )
    tui = ServoTui(
        controller, found or [args.min_id],
        id_candidates=list(range(args.min_id, args.max_id + 1)),
    )
    tui.connected = bool(found)
    tui.run()
    return 0


def command_set_zero(args, controller):
    """Re-label the servo's current pose.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    servo_id = resolve_servo_id(controller, args.id)
    if args.position is not None:
        wanted = args.position
    elif args.degree is not None:
        wanted = degree_to_position(args.degree)
    else:
        wanted = CENTER_POSITION

    before = controller.read_register(servo_id, "present_position")
    print(
        f"Servo {servo_id} currently reports {before} "
        f"({position_to_degree(before):+.1f} deg)."
    )
    if not confirm(
        f"Call this pose {wanted} ({position_to_degree(wanted):+.1f} deg)? "
        f"This writes homing_offset to the servo EEPROM. The servo will not "
        f"move.",
        args.yes,
    ):
        print("Cancelled.")
        return 1

    reported = controller.set_zero(servo_id, wanted)
    offset = controller.read_register(servo_id, "homing_offset")
    print(
        f"{Fore.GREEN}Servo {servo_id} now reports {reported} "
        f"({position_to_degree(reported):+.1f} deg) in the same pose."
        f"{Style.RESET_ALL}"
    )
    print(f"homing_offset = {offset}")
    if reported != wanted:
        print(
            f"{Fore.RED}Expected {wanted} but the servo reports {reported}."
            f"{Style.RESET_ALL}"
        )
        return 1
    return 0


def command_selftest(args, controller):
    """Check that the servo acts on the positions it is sent.

    Parameters
    ----------
    args : argparse.Namespace
        Parsed arguments.
    controller : FeetechServoController
        An open controller.

    Returns
    -------
    int
        Process exit status.
    """
    servo_id = resolve_servo_id(controller, args.id)
    print(f"Self test on servo {servo_id}. The servo will move.\n")

    print("Settings that would stop it moving:")
    report = controller.selftest(
        servo_id, span=args.span, moves=args.moves,
        progress=lambda result: None,
    )
    for name, ok, detail in report["checks"]:
        mark = f"{Fore.GREEN}ok  {Style.RESET_ALL}" if ok else f"{Fore.RED}FAIL{Style.RESET_ALL}"
        print(f"  [{mark}] {name}: {detail}")

    print(f"\nMoves, starting from {report['start']}:")
    print(f"  {'target':>7}  {'before':>7}  {'reached':>7}  {'error':>6}  "
          f"{'secs':>5}  result")
    for result in report["moves"]:
        if not result["commanded_move"]:
            verdict = "skipped, already there"
        elif result["reached_target"]:
            verdict = f"{Fore.GREEN}moved and reached the target{Style.RESET_ALL}"
        else:
            verdict = f"{Fore.RED}DID NOT REACH{Style.RESET_ALL}"
        print(
            f"  {result['target']:>7}  {result['before']:>7}  "
            f"{result['reached']:>7}  {result['error']:>6}  "
            f"{result['seconds']:>5.2f}  {verdict}"
        )

    print()
    if report["passed"]:
        print(
            f"{Fore.GREEN}PASS{Style.RESET_ALL}: "
            f"{report['moved_count']} of {report['asked_count']} commanded "
            f"moves reached their target. The servo acts on what it is sent."
        )
        return 0
    print(
        f"{Fore.RED}FAIL{Style.RESET_ALL}: the servo did not follow the "
        f"commands above."
    )
    _print_stuck_hints()
    return 1


def _print_stuck_hints():
    """Print what to look at when a servo will not move."""
    print()
    print("Things that keep a servo still even with torque on:")
    print("  - operating mode is 'wheel': goal_position is ignored, it takes")
    print("    a velocity instead. Fix with 'feetech mode <id> position'.")
    print("  - torque_limit or max_torque_limit is 0.")
    print("  - min/max position limit exclude the target.")
    print("  - the supply cannot deliver the stall current, so the servo")
    print("    browns out and resets. Check present_voltage under load.")
    print("  - a hardware error latched in the status register; power cycle.")


#: Sub commands that need an open serial bus, and the function implementing them.
BUS_COMMANDS = {
    "scan": command_scan,
    "info": command_info,
    "set-id": command_set_id,
    "set-baud": command_set_baud,
    "read": command_read,
    "write": command_write,
    "move": command_move,
    "torque": command_torque,
    "mode": command_mode,
    "factory-reset": command_factory_reset,
    "set-zero": command_set_zero,
    "selftest": command_selftest,
    "monitor": command_monitor,
}


def main(argv=None):
    """Run the command line interface.

    Parameters
    ----------
    argv : list of str or None, optional
        Command line arguments, defaults to ``sys.argv[1:]``.

    Returns
    -------
    int
        Process exit status.
    """
    colorama.init()
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "ports":
        return command_ports(args)
    if args.command == "registers":
        return command_registers(args)

    command = args.command or "monitor"
    if command not in BUS_COMMANDS:
        parser.error(f"unknown command: {command}")

    controller = make_controller(args)
    if args.simulate:
        attach_simulator(controller, args)
    try:
        # The interactive screen can start with a quiet bus and wait; the one
        # shot commands have nothing to wait for.
        controller.open(require_servo=command != "monitor")
    except FeetechError as error:
        print(f"{Fore.RED}{error}{Style.RESET_ALL}")
        _print_connection_hints()
        return 1

    try:
        return BUS_COMMANDS[command](args, controller)
    except FeetechError as error:
        print(f"{Fore.RED}{error}{Style.RESET_ALL}")
        return 1
    finally:
        controller.close()


def _print_empty_bus(controller):
    """Report that the bus is open but nothing answered on it.

    Parameters
    ----------
    controller : FeetechServoController
        An open controller.
    """
    speed = format_baud(controller.baudrate)
    print(
        f"{Fore.RED}No servo answered at {speed}bps "
        f"on {controller.serial.port}.{Style.RESET_ALL}"
    )
    print("Try 'feetech scan --all-baudrates' or widen --max-id.")


def _print_connection_hints():
    """Print troubleshooting advice after a failed connection."""
    print()
    print("Check that:")
    print("  1. the USB servo adapter is plugged in ('feetech ports' lists it)")
    print("  2. the servo has its own power supply, USB alone is not enough")
    print("  3. the data line is wired to the bus, not swapped with ground")
    print(f"  4. --max-id covers the servo id (default {DEFAULT_MAX_ID})")
    print()
    print("'feetech scan --all-baudrates' probes every supported bus speed.")


if __name__ == "__main__":
    sys.exit(main())
