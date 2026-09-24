"""End to end tests driving main() against a simulated bus."""

import pytest
from fake_servo import FakeBus
from fake_servo import FakeServo

from feetech_cli import cli
from feetech_cli import controller as controller_module
from feetech_cli.registers import BAUDRATE_INDEX


@pytest.fixture
def bus(monkeypatch):
    """Replace :class:`serial.Serial` with a simulated bus.

    Returns
    -------
    FakeBus
        The bus the CLI ends up talking to. Its servos survive reopens, so
        speed changes behave the way they do on real hardware.
    """
    servos = [FakeServo(1), FakeServo(4)]
    fake = FakeBus(servos)

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
    """Run the CLI and return its exit status.

    Parameters
    ----------
    argv : list of str
        Arguments after the program name.

    Returns
    -------
    int
        Process exit status.
    """
    return cli.main(["--port", "/dev/fake", "--baudrate", "1000000"] + argv)


def test_scan_lists_every_servo(bus, capsys):
    assert run(["scan"]) == 0
    out = capsys.readouterr().out
    assert "Found 2 servo(s)" in out
    assert "id   1" in out
    assert "id   4" in out


def test_info_prints_the_control_table(bus, capsys):
    assert run(["info", "1"]) == 0
    out = capsys.readouterr().out
    assert "baud_rate" in out
    assert "1.00Mbps" in out
    assert "present_position" in out


def test_info_needs_an_id_when_several_servos_answer(bus, capsys):
    assert run(["info"]) == 1
    assert "several servos answered" in capsys.readouterr().out


def test_set_id_changes_the_servo(bus, capsys):
    assert run(["set-id", "1", "9", "-y"]) == 0
    assert "now id 9" in capsys.readouterr().out
    assert sorted(servo.servo_id for servo in bus.servos) == [4, 9]


def test_set_id_refuses_a_taken_id(bus, capsys):
    assert run(["set-id", "1", "4", "-y"]) == 1
    assert "already used" in capsys.readouterr().out
    assert sorted(servo.servo_id for servo in bus.servos) == [1, 4]


def test_set_id_without_confirmation_is_cancelled(bus, monkeypatch, capsys):
    monkeypatch.setattr(cli.sys.stdin, "isatty", lambda: False)
    assert run(["set-id", "1", "9"]) == 1
    assert "Cancelled" in capsys.readouterr().out
    assert bus.servos[0].servo_id == 1


def test_set_baud_changes_the_speed(bus, capsys):
    assert run(["set-baud", "1", "115200", "-y"]) == 0
    assert "115kbps" in capsys.readouterr().out
    assert bus.servos[0].get("baud_rate") == BAUDRATE_INDEX[115200]


def test_set_baud_rejects_an_unsupported_speed(bus, capsys):
    assert run(["set-baud", "1", "9600", "-y"]) == 1
    assert "unsupported baud rate" in capsys.readouterr().out


def test_move_by_degree(bus, capsys):
    assert run(["move", "1", "--degree", "90"]) == 0
    assert bus.servos[0].get("goal_position") == 3072


def test_move_by_position(bus, capsys):
    assert run(["move", "4", "--position", "1000"]) == 0
    assert bus.servos[1].get("goal_position") == 1000


def test_torque_toggle(bus, capsys):
    assert run(["torque", "1", "on"]) == 0
    assert bus.servos[0].get("torque_enable") == 1
    assert run(["torque", "1", "off"]) == 0
    assert bus.servos[0].get("torque_enable") == 0


def test_mode_switches_to_wheel(bus, capsys):
    assert run(["mode", "1", "wheel"]) == 0
    assert bus.servos[0].get("operating_mode") == 1
    assert bus.servos[0].locked is True


def test_read_and_write_a_register(bus, capsys):
    assert run(["write", "1", "acceleration", "20"]) == 0
    assert bus.servos[0].get("acceleration") == 20
    assert run(["read", "1", "acceleration"]) == 0
    assert "acceleration = 20" in capsys.readouterr().out


def test_write_redirects_id_and_baud_to_the_safe_commands(bus, capsys):
    assert run(["write", "1", "id", "9"]) == 1
    assert "feetech set-id" in capsys.readouterr().out
    assert bus.servos[0].servo_id == 1


def test_scan_respects_max_id(bus, capsys):
    assert cli.main(
        ["--port", "/dev/fake", "--baudrate", "1000000", "--max-id", "3", "scan"]
    ) == 0
    out = capsys.readouterr().out
    assert "Found 1 servo(s)" in out


def test_registers_command_needs_no_hardware(capsys):
    assert cli.main(["registers"]) == 0
    assert "goal_position" in capsys.readouterr().out
