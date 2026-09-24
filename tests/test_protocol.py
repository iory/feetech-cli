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
