"""Tests for the high level controller against a simulated bus."""

import pytest
from fake_servo import FakeBus
from fake_servo import FakeServo

from feetech_cli.controller import FeetechServoController
from feetech_cli.protocol import FeetechError
from feetech_cli.registers import BAUDRATE_INDEX


def make_controller(servos, baudrate=1000000, echo=False):
    """Build a controller wired to a simulated bus.

    Parameters
    ----------
    servos : sequence of FakeServo
        Servos on the bus.
    baudrate : int, optional
        Initial bus speed.
    echo : bool, optional
        Whether the adapter echoes transmissions.

    Returns
    -------
    tuple of (FeetechServoController, FakeBus)
        The opened controller and the bus behind it.
    """
    bus = FakeBus(servos, baudrate=baudrate, echo=echo)
    controller = FeetechServoController(port="/dev/fake", baudrate=baudrate)

    def open_port(device, rate):
        """Swap in the simulated bus instead of a real serial port."""
        bus.baudrate = rate
        bus.is_open = True
        controller.serial = bus
        controller.baudrate = rate
        from feetech_cli.protocol import PacketHandler
        controller.packet_handler = PacketHandler(bus, little_endian=True)

    controller._open_port = open_port
    controller.open()
    return controller, bus


def test_scan_finds_servos_within_the_default_range():
    controller, _ = make_controller([FakeServo(1), FakeServo(4), FakeServo(16)])
    assert controller.scan() == [1, 4, 16]


def test_scan_default_range_stops_at_16():
    controller, _ = make_controller([FakeServo(1), FakeServo(17)])
    assert controller.scan() == [1]
    assert controller.scan(max_id=20) == [1, 17]


def test_read_register_decodes_sign_magnitude():
    servo = FakeServo(1)
    servo._store("homing_offset", 0x0800 | 100)  # sign bit 11 set
    controller, _ = make_controller([servo])
    assert controller.read_register(1, "homing_offset") == -100


def test_write_register_unlocks_and_relocks_eeprom():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    assert servo.locked is True
    assert controller.write_register(1, "min_position_limit", 500) == 500
    assert servo.locked is True
    assert servo.rejected_writes == []


def test_write_register_refuses_read_only_registers():
    controller, _ = make_controller([FakeServo(1)])
    with pytest.raises(FeetechError):
        controller.write_register(1, "present_position", 10)


def test_set_id_changes_the_servo_and_relocks():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    assert controller.set_id(1, 7) == 7
    assert servo.servo_id == 7
    assert servo.locked is True
    assert controller.scan() == [7]


def test_set_id_refuses_an_id_already_on_the_bus():
    controller, _ = make_controller([FakeServo(1), FakeServo(2)])
    with pytest.raises(FeetechError, match="already used"):
        controller.set_id(1, 2)


def test_set_id_refuses_an_absent_servo():
    controller, _ = make_controller([FakeServo(1)])
    with pytest.raises(FeetechError, match="no servo answered"):
        controller.set_id(9, 3)


def test_set_id_rejects_out_of_range_ids():
    controller, _ = make_controller([FakeServo(1)])
    with pytest.raises(FeetechError, match="between 0 and 253"):
        controller.set_id(1, 254)


def test_set_id_disables_torque_first():
    servo = FakeServo(1)
    servo._store("torque_enable", 1)
    controller, _ = make_controller([servo])
    controller.set_id(1, 3)
    assert servo.get("torque_enable") == 0


def test_set_baudrate_reopens_the_port_and_verifies():
    servo = FakeServo(1)
    controller, bus = make_controller([servo])
    assert controller.set_baudrate(1, 115200) == 115200
    assert servo.get("baud_rate") == BAUDRATE_INDEX[115200]
    assert bus.baudrate == 115200
    assert controller.ping(1) is True


def test_set_baudrate_rejects_unsupported_speeds():
    controller, _ = make_controller([FakeServo(1)])
    with pytest.raises(FeetechError, match="unsupported baud rate"):
        controller.set_baudrate(1, 9600)


def test_scan_all_baudrates_reports_each_speed():
    controller, _ = make_controller(
        [FakeServo(1, baud_index=0), FakeServo(2, baud_index=BAUDRATE_INDEX[115200])]
    )
    results = controller.scan_all_baudrates()
    assert results == {1000000: [1], 115200: [2]}


def test_silent_servo_is_detected_and_still_writable():
    servo = FakeServo(1, response_level=0)
    controller, _ = make_controller([servo])
    assert controller.detect_response_level(1) == 0
    assert 1 in controller.silent_servos
    controller.set_goal_position(1, 3000)
    assert servo.get("goal_position") == 3000


def test_goal_position_write_energises_a_limp_servo():
    """Measured on an STS3215: a position command turns torque on by itself.

    With torque explicitly off, writing goal_position took torque_enable from
    0 to 1 and the horn moved. Writes to lock, homing_offset and acceleration
    did not do this, so it is specific to the position command.
    """
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    assert controller.read_register(1, "torque_enable") == 0
    controller.set_goal_position(1, 1000)
    assert controller.read_register(1, "torque_enable") == 1
    assert controller.read_register(1, "goal_position") == 1000
    assert controller.read_register(1, "present_position") == 1000


def test_goal_position_moves_the_servo_once_torque_is_on():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.set_torque(1, True)
    controller.set_goal_position(1, 1000)
    assert controller.read_register(1, "present_position") == 1000


def test_factory_reset_restores_defaults():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.set_id(1, 5)
    assert servo.servo_id == 5
    controller.factory_reset(5)
    assert servo.servo_id == 1


def test_dump_reports_none_for_unreadable_registers():
    controller, _ = make_controller([FakeServo(1)])
    values = controller.dump(1, ["present_position", "present_voltage"])
    assert values["present_position"] == 2048
    assert values["present_voltage"] == 121


def test_selftest_passes_on_a_healthy_servo():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    report = controller.selftest(1, span=100, moves=4)
    assert report["passed"] is True
    assert report["asked_count"] >= 4
    assert report["moved_count"] == report["asked_count"]
    assert all(ok for _, ok, _ in report["checks"])


def test_selftest_leaves_the_servo_as_it_found_it():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    start = controller.read_register(1, "present_position")
    assert controller.read_register(1, "torque_enable") == 0
    controller.selftest(1, span=100, moves=4)
    assert controller.read_register(1, "present_position") == start
    assert controller.read_register(1, "torque_enable") == 0


def test_selftest_flags_wheel_mode():
    """Wheel mode ignores goal_position, which is a classic 'it will not move'."""
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.write_register(1, "operating_mode", 1)
    report = controller.selftest(1, span=100, moves=2)
    failed = [name for name, ok, _ in report["checks"] if not ok]
    assert "operating mode is position" in failed
    assert report["passed"] is False


def test_selftest_flags_zero_torque_limit():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.write_register(1, "torque_limit", 0)
    report = controller.selftest(1, span=100, moves=2)
    failed = [name for name, ok, _ in report["checks"] if not ok]
    assert "torque limit above zero" in failed
    assert report["passed"] is False


def test_set_zero_relabels_the_pose_without_moving_it():
    """Zeroing changes the number reported, not where the horn is."""
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    # Put the horn somewhere off centre.
    controller.set_torque(1, True)
    controller.set_goal_position(1, 1200)
    controller.set_torque(1, False)
    raw_before = servo.raw_position

    assert controller.read_register(1, "present_position") == 1200
    assert controller.set_zero(1) == 2048
    assert controller.read_register(1, "present_position") == 2048
    # The physical encoder did not move, only the label changed.
    assert servo.raw_position == raw_before


def test_set_zero_can_name_a_different_angle():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.set_torque(1, True)
    controller.set_goal_position(1, 1200)
    controller.set_torque(1, False)
    raw_before = servo.raw_position
    assert controller.set_zero(1, position=3000) == 3000
    assert servo.raw_position == raw_before


def test_set_zero_points_the_goal_at_the_new_reading():
    """Re-enabling torque after zeroing must not make the servo jump."""
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.set_torque(1, True)
    controller.set_goal_position(1, 1200)
    controller.set_torque(1, False)
    controller.set_zero(1)
    assert controller.read_register(1, "goal_position") == 2048
    raw_before = servo.raw_position
    controller.set_torque(1, True)
    servo._settle()
    assert servo.raw_position == raw_before


def test_set_zero_refuses_an_offset_the_register_cannot_hold():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.set_torque(1, True)
    controller.set_goal_position(1, 0)
    controller.set_torque(1, False)
    with pytest.raises(FeetechError, match="only holds"):
        controller.set_zero(1, position=4000)


def test_present_position_wraps_when_the_offset_crosses_zero():
    """Measured on an STS3215: raw 628 with an offset of 904 reads 3820.

    The reported position stays inside one turn rather than going negative,
    which is what makes the raw reading ambiguous until it is unwrapped.
    """
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    servo.raw_position = 628
    servo._store("homing_offset", 904)
    servo._settle()

    assert controller.read_register(1, "present_position") == 3820


def test_set_zero_works_when_the_reported_position_has_wrapped():
    """A servo that already carries an offset can still be zeroed.

    The offset needed here is -1420, well inside the +/-2047 the register
    holds. Working it out from the unwrapped reading asks for 2676 instead,
    and refuses a zeroing that is perfectly representable.
    """
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    servo.raw_position = 628
    servo._store("homing_offset", 904)
    servo._settle()
    raw_before = servo.raw_position

    assert controller.set_zero(1) == 2048
    assert controller.read_register(1, "homing_offset") == -1420
    assert controller.read_register(1, "present_position") == 2048
    # The horn did not move, only the label changed.
    assert servo.raw_position == raw_before


def test_set_zero_leaves_torque_off():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.set_zero(1)
    assert controller.read_register(1, "torque_enable") == 0


def test_sync_write_positions_moves_every_servo_in_one_packet():
    servos = [FakeServo(1), FakeServo(2), FakeServo(3)]
    controller, bus = make_controller(servos)
    for servo in servos:
        controller.set_torque(servo.servo_id, True)
    before = len(bus.tx_log)
    controller.sync_write_positions([1, 2, 3], [1000, 2000, 3000])
    assert len(bus.tx_log) - before == 1          # one packet, not three
    assert [s.get("goal_position") for s in servos] == [1000, 2000, 3000]
    assert [s.get("present_position") for s in servos] == [1000, 2000, 3000]


def test_sync_write_positions_carries_speed_and_acceleration():
    servo = FakeServo(1)
    controller, _ = make_controller([servo])
    controller.sync_write_positions([1], [1500], velocity=400, acceleration=30)
    assert servo.get("goal_position") == 1500
    assert servo.get("goal_velocity") == 400
    assert servo.get("acceleration") == 30


def test_sync_write_positions_skips_servos_that_are_not_there():
    servos = [FakeServo(1), FakeServo(5)]
    controller, _ = make_controller(servos)
    controller.sync_write_positions([1, 3, 5], [900, 900, 900])
    assert servos[0].get("goal_position") == 900
    assert servos[1].get("goal_position") == 900


def test_sync_write_rejects_ragged_entries():
    controller, _ = make_controller([FakeServo(1)])
    with pytest.raises(ValueError, match="same length"):
        controller.packet_handler.sync_write(42, [(1, [0, 0]), (2, [0])])
