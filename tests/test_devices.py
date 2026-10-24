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
from feetech_cli.protocol import INST_PING
from feetech_cli.protocol import INST_WRITE
from feetech_cli.protocol import FeetechError
from feetech_cli.protocol import PacketHandler
from feetech_cli.protocol import checksum

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


def reader(memory):
    """A ``read(address, length)`` over a byte string, logging each call."""
    calls = []

    def read(address, length):
        """Return a slice of ``memory``."""
        calls.append((address, length))
        return memory[address:address + length]

    return read, calls


def test_identify_needs_the_match_bytes(profile):
    assert identify(reader(bytes.fromhex("3412455842"))[0], [profile]) is profile
    # A servo's first bytes: firmware 3.10, model 777.
    assert identify(reader(bytes([3, 10, 0, 0x09, 0x03]))[0], [profile]) is SERVO
    assert identify(reader(bytes.fromhex("34124558ff"))[0], [profile]) is SERVO
    assert identify(reader(b"")[0], [profile]) is SERVO


def _variant(revision, name):
    """The example board, told apart from its sibling by a revision byte."""
    data = json.loads(EXAMPLE.read_text())
    data["name"] = name
    data["match"] = [data["match"], {"address": 40, "bytes": f"{revision:02x}"}]
    return profile_from_dict(data)


def test_a_list_match_tells_boards_with_one_model_number_apart():
    rev2, rev3 = _variant(2, "Board rev 2"), _variant(3, "Board rev 3")
    memory = bytearray(64)
    memory[0:5] = bytes.fromhex("3412455842")
    memory[40] = 3
    read, calls = reader(bytes(memory))
    assert identify(read, [rev2, rev3]) is rev3
    # The shared model bytes are read once, not once per device file.
    assert calls == [(0, 5), (40, 1)]


def test_a_servo_costs_one_read_however_long_the_match_list():
    servo_memory = bytes([3, 10, 0, 0x09, 0x03]) + bytes(59)
    read, calls = reader(servo_memory)
    assert identify(read, [_variant(2, "Board rev 2"), _variant(3, "Board rev 3")]) is SERVO
    assert calls == [(0, 5)]


def test_the_simulated_board_answers_to_every_match_entry():
    rev3 = _variant(3, "Board rev 3")
    controller, _ = make_controller(
        [FakeServo(1), FakeBoard(rev3, board_id=BOARD_ID)], [_variant(2, "Board rev 2"), rev3]
    )
    assert controller.device_profile(BOARD_ID) is rev3
    assert controller.is_servo(1)


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


def _with_settle(seconds):
    """The example board, quiet for ``seconds`` after a write to 'mode'."""
    data = json.loads(EXAMPLE.read_text())
    data["registers"]["mode"]["settle_time"] = seconds
    return profile_from_dict(data)


def test_a_write_with_a_settle_time_waits_for_the_device_before_reading_back(monkeypatch):
    profile = _with_settle(1.5)
    controller, _ = make_controller([make_board(profile)], [profile])
    waited = []
    real_wait = controller._wait_for

    def wait_for(servo_id, settle_time):
        """Record the wait, then ping as usual."""
        waited.append((servo_id, settle_time))
        return real_wait(servo_id, settle_time)

    monkeypatch.setattr(controller, "_wait_for", wait_for)
    assert controller.write_register(BOARD_ID, "mode", 1) == 1
    assert waited == [(BOARD_ID, 1.5)]
    # Registers without one are read back straight away.
    waited.clear()
    controller.write_register(BOARD_ID, "id", BOARD_ID)
    assert waited == []


def test_late_replies_to_buffered_pings_are_not_read_back_as_the_register(monkeypatch):
    # A board erasing flash buffers the pings it cannot answer and, once
    # back, answers them all; the extra replies are still on the wire when
    # the read-back goes out.
    profile = _with_settle(1.0)
    controller, bus = make_controller([make_board(profile)], [profile])
    stale_reply = bytes([0xFF, 0xFF, BOARD_ID, 2, 0, checksum([BOARD_ID, 2, 0])])
    late = bytearray()
    answered = []
    real_write = bus.write
    real_read = bus.read
    real_reset = bus.reset_input_buffer

    def arrive():
        """The late replies land in the receive buffer."""
        bus.rx.extend(late)
        late.clear()

    def write(packet):
        """After the first ping that follows the write, queue late replies."""
        real_write(packet)
        if packet[4] == INST_PING and written_addresses(bus, BOARD_ID) and not answered:
            answered.append(packet)
            late.extend(stale_reply * 14)

    def read(count=1):
        """Late bytes show up once the buffer has been read dry."""
        if not bus.rx:
            arrive()
        return real_read(count)

    def reset_input_buffer():
        """A flush drops what is there; the late replies are still coming."""
        real_reset()
        arrive()

    monkeypatch.setattr(bus, "write", write)
    monkeypatch.setattr(bus, "read", read)
    monkeypatch.setattr(bus, "reset_input_buffer", reset_input_buffer)
    assert controller.write_register(BOARD_ID, "mode", 1) == 1


def test_a_device_that_stays_silent_past_its_settle_time_is_an_error(monkeypatch):
    profile = _with_settle(0.2)
    controller, _ = make_controller([make_board(profile)], [profile])
    monkeypatch.setattr(controller, "_wait_for", lambda servo_id, settle_time: False)
    with pytest.raises(FeetechError, match="did not answer again within 0.2 s"):
        controller.write_register(BOARD_ID, "mode", 1)


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
        (lambda d: d.update(match=[]), "needs a 'match'"),
        (lambda d: d["match"].update(adress=1), r"'match' has unknown key\(s\) \['adress'\]"),
        (lambda d: d.update(nmae="x"), r"unknown key\(s\) \['nmae'\]"),
        (lambda d: d["registers"]["mode"].update(adress=3), "register 'mode' has unknown key"),
        (lambda d: d["registers"]["mode"].update(size=4), "'size' must be 1 or 2"),
        (lambda d: d["registers"]["model_number"].update(settle_time=1),
         "'settle_time' must be a non-negative number of seconds on a writable register"),
        (lambda d: d["registers"]["mode"].update(settle_time=-1), "'settle_time' must be"),
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

    def fake_serial(device, baudrate, timeout=None, exclusive=None):
        """Hand back the same bus whatever device is opened."""
        fake.port = device
        fake.baudrate = baudrate
        fake.timeout = timeout
        fake.exclusive = exclusive
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


def test_read_registers_reads_a_run_in_one_instruction(profile):
    from feetech_cli.protocol import INST_READ

    controller, bus = make_controller([make_board(profile)], [profile])
    controller.device_profile(BOARD_ID)
    before = len(bus.tx_log)
    values = controller.read_registers(BOARD_ID, ["acc_z", "pressure", "acc_range_g", "mode"])
    reads = [p for p in bus.tx_log[before:] if p[4] == INST_READ]
    assert len(reads) == 1
    assert values == {"acc_z": 2048, "pressure": -4210, "acc_range_g": 16, "mode": 0}


def test_watch_prints_one_line_per_refresh_off_a_terminal(mixed_bus, capsys):
    assert run(["watch", str(BOARD_ID), "acc_z", "pressure", "-n", "3", "-i", "0"]) == 0
    lines = capsys.readouterr().out.strip().splitlines()
    assert lines[-3:] == ["acc_z=2048 (+1.000 g)  pressure=-4210 (-42.10 kPa)"] * 3


def test_watch_rejects_a_register_the_device_lacks(mixed_bus, capsys):
    assert run(["watch", str(BOARD_ID), "present_position", "-n", "1"]) == 1
    assert "Unknown register(s) present_position" in capsys.readouterr().out
