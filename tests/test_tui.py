"""Tests for the interactive screen, driven without a terminal."""

import pytest
import readchar
from fake_servo import FakeBus
from fake_servo import FakeServo

from feetech_cli.cli import build_parser
from feetech_cli.controller import FeetechServoController
from feetech_cli.protocol import FeetechError
from feetech_cli.protocol import PacketHandler
from feetech_cli.registers import CENTER_POSITION
from feetech_cli.tui import POSITION_STEP
from feetech_cli.tui import SELECTABLE_ROWS
from feetech_cli.tui import ServoTui


@pytest.fixture
def tui():
    """Build a TUI wired to a simulated two servo bus.

    Returns
    -------
    tuple of (ServoTui, list of FakeServo)
        The screen and the servos behind it.
    """
    servos = [FakeServo(1), FakeServo(4)]
    bus = FakeBus(servos)
    controller = FeetechServoController(port="/dev/fake", baudrate=1000000)
    controller.serial = bus
    controller.baudrate = 1000000
    controller.packet_handler = PacketHandler(bus, little_endian=True)
    screen = ServoTui(controller, [1, 4])
    screen.sync_from_servo()
    return screen, servos


def test_draw_renders_every_row(tui, capsys):
    screen, _ = tui
    screen.draw()
    out = capsys.readouterr().out
    for label in ["Servo ID", "Goal Position", "Present Position", "Voltage",
                  "Temperature", "Torque", "Operating Mode", "Baud Rate", "EEPROM"]:
        assert label in out
    assert "12.1 V" in out
    assert "1.00Mbps" in out


def test_arrow_keys_move_the_cursor(tui):
    screen, _ = tui
    assert screen.selected_row == 0
    screen.handle_key(readchar.key.DOWN)
    assert SELECTABLE_ROWS[screen.selected_row] == "Goal Position"
    screen.handle_key(readchar.key.UP)
    assert SELECTABLE_ROWS[screen.selected_row] == "Servo ID"


def test_right_arrow_commands_a_new_goal(tui):
    screen, servos = tui
    screen.selected_row = SELECTABLE_ROWS.index("Goal Position")
    before = servos[0].get("goal_position")
    screen.handle_key(readchar.key.RIGHT)
    assert servos[0].get("goal_position") > before


def test_right_arrow_says_it_energised_a_limp_servo(tui):
    """The arrows move the servo even from a standstill, and say so.

    A goal_position write switches the output stage on by itself, so the joint
    stops being back drivable the moment an arrow is pressed.
    """
    screen, servos = tui
    assert servos[0].get("torque_enable") == 0
    screen.selected_row = SELECTABLE_ROWS.index("Goal Position")
    screen.handle_key(readchar.key.RIGHT)
    assert "energised the servo" in screen.message
    assert servos[0].get("torque_enable") == 1
    assert servos[0].get("present_position") > 2048


def test_right_arrow_moves_the_servo_once_torque_is_on(tui):
    screen, servos = tui
    screen.handle_key("t")
    screen.read_values()
    screen.selected_row = SELECTABLE_ROWS.index("Goal Position")
    screen.handle_key(readchar.key.RIGHT)
    assert "Torque is off" not in screen.message
    assert servos[0].get("present_position") == servos[0].get("goal_position")
    assert servos[0].get("present_position") > 2048


def test_repeated_presses_accumulate(tui):
    screen, servos = tui
    screen.selected_row = SELECTABLE_ROWS.index("Goal Position")
    for _ in range(3):
        screen.handle_key(readchar.key.RIGHT)
    assert servos[0].get("goal_position") == 2048 + 3 * POSITION_STEP


def test_enabling_torque_holds_the_current_position(tui):
    """A stale goal must not make the servo snap when torque comes back on."""
    screen, servos = tui
    servos[0]._store("goal_position", 3500)
    servos[0]._store("present_position", 2048)
    screen.read_values()
    screen.handle_key("t")
    assert servos[0].get("torque_enable") == 1
    assert servos[0].get("goal_position") == 2048
    assert servos[0].get("present_position") == 2048
    assert "holding the current position" in screen.message


def test_centre_key_recentres(tui):
    screen, servos = tui
    screen.handle_key("z")
    assert servos[0].get("goal_position") == CENTER_POSITION


def test_torque_key_toggles(tui):
    screen, servos = tui
    assert servos[0].get("torque_enable") == 0
    screen.handle_key("t")
    assert servos[0].get("torque_enable") == 1
    screen.handle_key("t")
    assert servos[0].get("torque_enable") == 0


def test_present_position_says_when_the_horn_is_held(tui):
    """An energised servo springs back, so hand movement will not stick."""
    screen, _ = tui
    assert "held by the servo" not in screen.render_row(
        "Present Position", screen.values
    )
    screen.handle_key("t")
    screen.read_values()
    assert "held by the servo" in screen.render_row(
        "Present Position", screen.values
    )


def test_next_and_previous_switch_servo(tui):
    screen, _ = tui
    assert screen.servo_id == 1
    screen.handle_key("n")
    assert screen.servo_id == 4
    screen.handle_key("p")
    assert screen.servo_id == 1


def test_enter_writes_the_highlighted_id(tui):
    screen, servos = tui
    screen.selected_row = SELECTABLE_ROWS.index("Servo ID")
    # Candidates are 1..16 and the cursor starts on the servo's own id.
    for _ in range(6):
        screen.handle_key(readchar.key.RIGHT)
    assert screen.id_candidates[screen.candidate_index] == 7
    screen.handle_key(readchar.key.ENTER)
    assert servos[0].servo_id == 7
    assert screen.servo_ids == [4, 7]
    assert screen.servo_id == 7


def test_enter_on_a_taken_id_reports_and_changes_nothing(tui):
    screen, servos = tui
    screen.selected_row = SELECTABLE_ROWS.index("Servo ID")
    screen.candidate_index = screen.id_candidates.index(4)
    with pytest.raises(Exception, match="already used"):
        screen.handle_key(readchar.key.ENTER)
    assert servos[0].servo_id == 1


def test_id_candidates_default_to_the_scan_range(tui):
    screen, _ = tui
    assert screen.id_candidates == list(range(1, 17))


def test_no_subparser_argument_shadows_a_global_option():
    """A sub command must not reuse a global option's namespace attribute.

    ``set-baud <id> <baudrate>`` once stored its positional over the global
    ``--baudrate``, which made the CLI open the port at the speed it was
    about to write.
    """
    parser = build_parser()
    globals_seen = {
        action.dest for action in parser._actions if action.dest != "help"
    }
    subparser_action = [
        action for action in parser._actions if hasattr(action, "choices")
        and isinstance(action.choices, dict)
    ][0]
    for name, subparser in subparser_action.choices.items():
        for action in subparser._actions:
            if action.dest in ("help", "command"):
                continue
            assert action.dest not in globals_seen, (
                f"'{name}' argument '{action.dest}' shadows a global option"
            )


def test_zero_key_asks_before_writing_eeprom(tui):
    screen, servos = tui
    servos[0]._store("goal_position", 1200)
    servos[0]._store("torque_enable", 1)
    servos[0]._settle()
    servos[0]._store("torque_enable", 0)
    screen.read_values()
    offset_before = servos[0].get("homing_offset")

    screen.handle_key("0")
    assert screen.pending_confirm is not None
    assert "writes homing_offset" in screen.pending_confirm[0]
    # Nothing written yet.
    assert servos[0].get("homing_offset") == offset_before


def test_zero_key_applies_on_y(tui):
    screen, servos = tui
    servos[0]._store("goal_position", 1200)
    servos[0]._store("torque_enable", 1)
    servos[0]._settle()
    servos[0]._store("torque_enable", 0)
    screen.read_values()
    raw_before = servos[0].raw_position

    screen.handle_key("0")
    screen.handle_key("y")
    assert screen.pending_confirm is None
    assert servos[0].get("present_position") == CENTER_POSITION
    # Re-labelled, not moved.
    assert servos[0].raw_position == raw_before
    assert "This pose is now 2048" in screen.message


def test_zero_key_cancels_on_any_other_key(tui):
    screen, servos = tui
    offset_before = servos[0].get("homing_offset")
    screen.handle_key("0")
    screen.handle_key("n")
    assert screen.pending_confirm is None
    assert servos[0].get("homing_offset") == offset_before
    assert "Cancelled" in screen.message
    # The cancelling key must not also act as 'next servo'.
    assert screen.servo_id == 1


def test_pending_question_is_shown_on_screen(tui):
    screen, _ = tui
    screen.handle_key("0")
    text = "\n".join(screen.build_lines(screen.values))
    assert "Press 'y' to confirm" in text


def test_losing_the_servo_shows_the_waiting_screen(tui):
    screen, servos = tui
    bus = screen.controller.serial
    bus.servos = []                       # servo unplugged
    with pytest.raises(FeetechError):
        screen.read_values()
    screen.lose_bus(FeetechError("gone"))
    assert screen.connected is False
    text = "\n".join(screen.build_lines({}))
    assert "No servo is answering" in text
    assert "picked up automatically" in text


def test_a_swapped_servo_is_picked_up(tui):
    """The whole point: change the servo and the screen follows it."""
    screen, servos = tui
    bus = screen.controller.serial
    screen.lose_bus(FeetechError("gone"))
    assert screen.connected is False

    bus.servos = [FakeServo(9)]           # a different servo, different id
    assert screen.rediscover() is True
    assert screen.servo_ids == [9]
    assert screen.servo_id == 9
    assert "Found servo ids: 9" in screen.message


def test_the_same_id_reconnects_without_fuss(tui):
    screen, servos = tui
    bus = screen.controller.serial
    screen.lose_bus(FeetechError("gone"))
    bus.servos = [FakeServo(1)]
    assert screen.rediscover() is True
    assert screen.servo_id == 1
    assert "Reconnected to servo 1" in screen.message


def test_rediscover_reports_failure_while_the_bus_is_empty(tui):
    screen, _ = tui
    screen.controller.serial.servos = []
    screen.lose_bus(FeetechError("gone"))
    assert screen.rediscover() is False
    assert screen.connected is False


def test_a_yanked_adapter_closes_the_port_for_reopening(tui):
    """An OSError means the device node went away, not just the servo."""
    screen, _ = tui
    screen.lose_bus(OSError("device not configured"))
    assert screen.connected is False
    assert screen.controller.is_open is False


def test_keys_are_ignored_while_disconnected(tui):
    screen, servos = tui
    screen.lose_bus(FeetechError("gone"))
    before = servos[0].get("goal_position")
    screen.selected_row = SELECTABLE_ROWS.index("Goal Position")
    screen.handle_key(readchar.key.RIGHT)
    screen.handle_key("t")
    screen.handle_key("0")
    assert servos[0].get("goal_position") == before
    assert screen.pending_confirm is None


def test_read_values_raises_when_every_register_fails(tui):
    """dump() maps failures to None; all of them None means the bus is gone."""
    screen, _ = tui
    screen.controller.serial.servos = []
    with pytest.raises(FeetechError):
        screen.read_values()


def test_starting_on_a_quiet_bus_does_not_claim_to_have_lost_one(tui):
    screen, _ = tui
    screen.controller.serial.servos = []
    screen.connected = False
    lines = "\n".join(screen.build_lines({}))
    assert "No servo is answering" in lines
    assert "Lost the servo" not in lines
