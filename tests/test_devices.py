"""A device described by a device file, sharing the bus with servos."""

import json
from pathlib import Path

import pytest
from fake_servo import FakeBoard
from fake_servo import FakeBus
from fake_servo import FakeServo

from feetech_cli import cli
from feetech_cli import controller as controller_module
from feetech_cli.controller import FeetechServoController
from feetech_cli.devices import DEVICES_ENV
from feetech_cli.devices import SERVO
from feetech_cli.devices import DeviceFileError
from feetech_cli.devices import identify
from feetech_cli.devices import load_profile
from feetech_cli.devices import load_profiles
from feetech_cli.devices import profile_from_dict
from feetech_cli.protocol import INST_WRITE
from feetech_cli.protocol import FeetechError
from feetech_cli.protocol import PacketHandler

BOARD_ID = 10
EXAMPLE = Path(__file__).parent / "data" / "example_board.json"


@pytest.fixture
def profile():
    """The example device file, loaded."""
    return load_profile(EXAMPLE)


def make_board(profile, board_id=BOARD_ID):
    """A simulated example board with a few readings filled in."""
    board = FakeBoard(profile, board_id=board_id)
    board.set_register("model_number", 0x1234)
    board.set_register("firmware_version", 2)
    board.set_register("acc_range_g", 16)
    board.set_register("acc_z", 2048)
    board.set_register("pressure", -4210)
    return board


def make_controller(devices, profiles, baudrate=1000000):
    """Build an open controller wired to a simulated bus.

    Parameters
    ----------
    devices : sequence
        Simulated servos and boards on the bus.
    profiles : sequence of DeviceProfile
        Loaded device files.
    baudrate : int, optional
        Initial bus speed.

    Returns
    -------
    tuple of (FeetechServoController, FakeBus)
        The controller and the bus behind it.
    """
    bus = FakeBus(devices, baudrate=baudrate)
    controller = FeetechServoController(
        port="/dev/fake", baudrate=baudrate, max_id=20, device_profiles=profiles
    )

    def open_port(device, rate):
        """Swap in the simulated bus instead of a real serial port."""
        bus.baudrate = rate
        bus.is_open = True
        controller.serial = bus
        controller.baudrate = rate
        controller.packet_handler = PacketHandler(bus, little_endian=True)

    controller._open_port = open_port
    controller.open()
    return controller, bus


def written_addresses(bus, device_id):
    """Addresses of every WRITE instruction sent to one id."""
    return [
        packet[5] for packet in bus.tx_log
        if packet[2] == device_id and packet[4] == INST_WRITE
    ]


def test_identify_needs_the_match_bytes(profile):
    assert identify(bytes.fromhex("3412455842"), [profile]) is profile
    # A servo's first bytes: firmware 3.10, model 777.
    assert identify(bytes([3, 10, 0, 0x09, 0x03]), [profile]) is SERVO
    assert identify(bytes.fromhex("34124558ff"), [profile]) is SERVO
    assert identify(b"", [profile]) is SERVO


def test_without_device_files_nothing_is_probed():
    controller, bus = make_controller([FakeServo(1)], profiles=[])
    assert controller.scan() == [1]
    before = len(bus.tx_log)
    assert controller.is_servo(1)
    assert len(bus.tx_log) == before


def test_profiles_are_identified_per_id(profile):
    controller, _ = make_controller([FakeServo(1), make_board(profile)], [profile])
    assert controller.scan() == [1, BOARD_ID]
    assert controller.is_servo(1)
    assert controller.device_profile(BOARD_ID) is profile


def test_board_registers_come_from_its_own_table(profile):
    controller, _ = make_controller([make_board(profile)], [profile])
    assert controller.read_register(BOARD_ID, "id") == BOARD_ID
    assert controller.read_register(BOARD_ID, "model_number") == 0x1234
    assert controller.read_register(BOARD_ID, "pressure") == -4210


def test_board_write_needs_no_eeprom_lock(profile):
    controller, bus = make_controller([make_board(profile)], [profile])
    assert controller.write_register(BOARD_ID, "mode", 1) == 1
    assert written_addresses(bus, BOARD_ID) == [20]


def test_a_refused_board_write_is_reported_as_a_refusal(profile):
    board = make_board(profile)
    board.writable = frozenset()
    controller, _ = make_controller([board], [profile])
    with pytest.raises(FeetechError, match="refused the write to 'mode'"):
        controller.write_register(BOARD_ID, "mode", 1)


def test_set_id_and_baud_use_the_boards_registers(profile):
    board = make_board(profile)
    controller, bus = make_controller([FakeServo(1), board], [profile])
    assert controller.set_id(BOARD_ID, 11) == 11
    assert board.servo_id == 11
    assert controller.set_baudrate(11, 500000) == 500000
    assert board.baudrate == 500000
    assert written_addresses(bus, BOARD_ID) == [7]
    assert written_addresses(bus, 11) == [8]


@pytest.mark.parametrize(
    "command",
    [
        lambda c: c.set_torque(BOARD_ID, True),
        lambda c: c.set_goal_position(BOARD_ID, 2048),
        lambda c: c.factory_reset(BOARD_ID),
        lambda c: c.sync_write_positions([1, BOARD_ID], [2048, 2048]),
    ],
)
def test_servo_commands_refuse_a_board(profile, command):
    board = make_board(profile)
    before = bytes(board.memory)
    controller, bus = make_controller([FakeServo(1), board], [profile])
    with pytest.raises(FeetechError, match="not a servo"):
        command(controller)
    assert bytes(board.memory) == before
    assert written_addresses(bus, BOARD_ID) == []


def _broken(tmp_path, data):
    """Write a device file and return its path."""
    path = tmp_path / "broken.json"
    path.write_text(json.dumps(data))
    return path


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda d: d.pop("match"), "needs a 'match'"),
        (lambda d: d.update(nmae="x"), r"unknown key\(s\) \['nmae'\]"),
        (lambda d: d["registers"]["mode"].update(adress=3), "register 'mode' has unknown key"),
        (lambda d: d["registers"]["mode"].update(size=4), "'size' must be 1 or 2"),
        (lambda d: d.update(info=["nope"]), r"'info' names unknown register\(s\) \['nope'\]"),
        (lambda d: d["registers"]["acc_z"]["scale_by"].update(register="nope"),
         "scales by unknown register 'nope'"),
    ],
)
def test_a_broken_device_file_names_the_problem(tmp_path, mutate, message):
    data = json.loads(EXAMPLE.read_text())
    mutate(data)
    with pytest.raises(DeviceFileError, match=message):
        load_profile(_broken(tmp_path, data))


def test_profiles_are_found_on_the_command_line_env_and_config_dir(tmp_path):
    config = tmp_path / "config" / "feetech-cli" / "devices"
    config.mkdir(parents=True)
    data = json.loads(EXAMPLE.read_text())
    (config / "from_config.json").write_text(json.dumps(data))
    env_dir = tmp_path / "env"
    env_dir.mkdir()
    (env_dir / "from_env.json").write_text(json.dumps(data))
    environ = {"XDG_CONFIG_HOME": str(tmp_path / "config"), DEVICES_ENV: str(env_dir)}

    profiles = load_profiles([str(EXAMPLE)], environ=environ)
    assert [p.key for p in profiles] == ["example_board", "from_env", "from_config"]


def test_a_missing_device_file_is_an_error_not_a_servo(tmp_path):
    with pytest.raises(DeviceFileError, match="cannot read"):
        load_profiles([str(tmp_path / "absent.json")], environ={}, search_default=False)


def test_profile_from_dict_accepts_a_custom_baud_table():
    data = json.loads(EXAMPLE.read_text())
    data["baudrate_table"] = {"1": 1000000, "3": 115200}
    profile = profile_from_dict(data)
    assert profile.baudrate_table == {1: 1000000, 3: 115200}
    assert profile.key == "example-sensor-board"


@pytest.fixture
def mixed_bus(monkeypatch, profile):
    """A servo and the example board behind a patched :class:`serial.Serial`."""
    fake = FakeBus([FakeServo(1), make_board(profile)])

    def fake_serial(device, baudrate, timeout=None):
        """Hand back the same bus whatever device is opened."""
        fake.port = device
        fake.baudrate = baudrate
        fake.timeout = timeout
        fake.is_open = True
        fake.rx = bytearray()
        return fake

    monkeypatch.setattr(controller_module.serial, "Serial", fake_serial)
    return fake


def run(argv):
    """Run the CLI against the mixed bus with the example device file."""
    return cli.main(
        ["--port", "/dev/fake", "--baudrate", "1000000", "--max-id", "20",
         "--device-file", str(EXAMPLE)] + argv
    )


def test_scan_labels_the_board(mixed_bus, capsys):
    assert run(["scan"]) == 0
    out = capsys.readouterr().out
    assert "Found 2 device(s)" in out
    assert "id  10  Example sensor board (model 0x1234, fw 2)" in out
    assert "mode 0 (off)" in out
    assert "pressure -4210 (-42.10 kPa)" in out


def test_info_renders_with_the_device_files_rules(mixed_bus, capsys):
    assert run(["info", str(BOARD_ID)]) == 0
    out = capsys.readouterr().out
    assert "Example sensor board 10" in out
    assert "-42.10 kPa" in out
    assert "+1.000 g" in out
    assert "0x00 (none)" in out
    assert "present_position" not in out


def test_registers_lists_a_device_files_table(mixed_bus, capsys):
    assert run(["registers", "--device", "example_board"]) == 0
    out = capsys.readouterr().out
    assert "pressure" in out and "goal_position" not in out


def test_registers_names_the_known_devices_when_one_is_missing(mixed_bus, capsys):
    assert run(["registers", "--device", "nope"]) == 1
    assert "Known: example_board, servo" in capsys.readouterr().out


def test_a_broken_device_file_stops_the_cli(tmp_path, capsys):
    path = _broken(tmp_path, {"name": "x"})
    assert cli.main(["--device-file", str(path), "registers"]) == 2
    assert "needs a 'match'" in capsys.readouterr().out
