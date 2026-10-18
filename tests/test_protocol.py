"""Tests for the packet layer."""

import pytest
from fake_servo import FakeBus
from fake_servo import FakeServo

from feetech_cli.protocol import FeetechChecksumError
from feetech_cli.protocol import FeetechStatusError
from feetech_cli.protocol import FeetechTimeoutError
from feetech_cli.protocol import PacketHandler
from feetech_cli.protocol import checksum
from feetech_cli.protocol import describe_error
from feetech_cli.protocol import from_sign_magnitude
from feetech_cli.protocol import to_sign_magnitude


def test_checksum_matches_reference_packet():
    # Read 2 bytes from address 56 of servo 1:
    # FF FF 01 04 02 38 02 BE
    body = [0x01, 0x04, 0x02, 0x38, 0x02]
    assert checksum(body) == 0xBE


def test_build_packet_layout():
    handler = PacketHandler(FakeBus([]))
    packet = handler.build_packet(1, 0x02, [56, 2])
    assert packet == bytes([0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE])


def test_sign_magnitude_roundtrip():
    for sign_bit in (10, 11, 15):
        limit = (1 << sign_bit) - 1
        for value in (-limit, -1, 0, 1, limit):
            encoded = to_sign_magnitude(value, sign_bit)
            assert encoded >= 0
            assert from_sign_magnitude(encoded, sign_bit) == value


def test_sign_magnitude_rejects_values_that_would_clobber_the_sign_bit():
    # 2047 needs 11 magnitude bits, so it cannot be stored under sign bit 10.
    with pytest.raises(ValueError):
        to_sign_magnitude(-2047, 10)
    with pytest.raises(ValueError):
        to_sign_magnitude(2047, 10)


def test_word_order_differs_between_series():
    little = PacketHandler(FakeBus([]), little_endian=True)
    big = PacketHandler(FakeBus([]), little_endian=False)
    assert little.encode_word(0x1234) == [0x34, 0x12]
    assert big.encode_word(0x1234) == [0x12, 0x34]
    assert little.decode_word([0x34, 0x12]) == 0x1234
    assert big.decode_word([0x12, 0x34]) == 0x1234


def test_ping_finds_only_the_servos_present():
    bus = FakeBus([FakeServo(1), FakeServo(5)])
    handler = PacketHandler(bus)
    assert handler.ping(1) is True
    assert handler.ping(5) is True
    assert handler.ping(2) is False


def test_read_returns_register_bytes():
    servo = FakeServo(3)
    handler = PacketHandler(FakeBus([servo]))
    data = handler.read(3, 56, 2)
    assert handler.decode(2, data) == 2048


def test_write_then_read_roundtrip():
    servo = FakeServo(3)
    handler = PacketHandler(FakeBus([servo]))
    handler.write(3, 42, handler.encode(2, 1234))
    assert servo.get("goal_position") == 1234


def test_transmit_echo_is_skipped():
    servo = FakeServo(1)
    handler = PacketHandler(FakeBus([servo], echo=True))
    # Without echo handling this would parse our own instruction as the answer.
    assert handler.ping(1) is True
    assert handler.decode(2, handler.read(1, 56, 2)) == 2048


def voltage_alarm_servo(servo_id):
    """A servo whose PING answer is byte for byte the PING request."""
    servo = FakeServo(servo_id)
    servo.error = 0x01  # voltage, same value as INST_PING
    return servo


@pytest.mark.parametrize("echo", [False, True])
def test_ping_answer_identical_to_the_request_is_not_dropped(echo):
    handler = PacketHandler(FakeBus([voltage_alarm_servo(1)], echo=echo))
    assert handler.ping(1) is True
    assert handler.echoes is echo


@pytest.mark.parametrize("echo", [False, True])
def test_echo_detection_does_not_invent_servos(echo):
    handler = PacketHandler(FakeBus([voltage_alarm_servo(1)], echo=echo))
    assert handler.ping(2) is False
    assert handler.ping(1) is True
    assert handler.ping(3) is False


def test_echo_is_probed_only_once():
    bus = FakeBus([voltage_alarm_servo(1)])
    handler = PacketHandler(bus)
    handler.ping(1)
    handler.ping(1)
    # Two pings and a single id read to learn the adapter does not echo.
    assert len(bus.tx_log) == 3


def test_timeout_when_nobody_answers():
    handler = PacketHandler(FakeBus([FakeServo(1)]))
    with pytest.raises(FeetechTimeoutError):
        handler.read(9, 56, 2)


def test_corrupt_checksum_is_rejected():
    bus = FakeBus([FakeServo(1)])
    handler = PacketHandler(bus)
    bus.write(handler.build_packet(1, 0x01))
    bus.rx[-1] ^= 0xFF
    with pytest.raises(FeetechChecksumError):
        handler._read_status_packet()


def test_hardware_error_byte_raises():
    servo = FakeServo(1)
    servo.error = 0x04
    handler = PacketHandler(FakeBus([servo]))
    with pytest.raises(FeetechStatusError) as excinfo:
        handler.read(1, 56, 2)
    assert "overheat" in str(excinfo.value)


def test_describe_error_names_flags():
    assert describe_error(0) == ["none"]
    assert describe_error(0x05) == ["voltage", "overheat"]


def test_sync_read_returns_each_servo_in_order():
    servos = [FakeServo(1), FakeServo(4), FakeServo(2)]
    handler = PacketHandler(FakeBus(servos))
    handler.write(4, 42, handler.encode(2, 1000))  # goal_position
    handler.write(1, 42, handler.encode(2, 3000))
    answers = handler.sync_read([4, 1], 42, 2)
    assert list(answers) == [4, 1]
    assert handler.decode(2, answers[4][1]) == 1000
    assert handler.decode(2, answers[1][1]) == 3000


def test_sync_read_hands_back_an_error_byte_instead_of_raising():
    servo = FakeServo(1)
    servo.error = 0x20
    handler = PacketHandler(FakeBus([servo, FakeServo(2)]))
    answers = handler.sync_read([1, 2], 56, 2)
    assert answers[1][0] == 0x20 and answers[2][0] == 0


def test_sync_read_names_the_device_that_did_not_answer():
    handler = PacketHandler(FakeBus([FakeServo(1)]))
    with pytest.raises(FeetechTimeoutError, match="no answer from id 7"):
        handler.sync_read([1, 7], 56, 2)


def test_sync_read_skips_the_transmit_echo():
    handler = PacketHandler(FakeBus([FakeServo(1), FakeServo(2)], echo=True))
    assert list(handler.sync_read([1, 2], 56, 2)) == [1, 2]


def test_sync_read_refuses_repeated_ids():
    handler = PacketHandler(FakeBus([FakeServo(1)]))
    with pytest.raises(ValueError):
        handler.sync_read([1, 1], 56, 2)
