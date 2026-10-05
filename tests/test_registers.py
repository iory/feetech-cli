"""Tests for register decoding and rendering."""

from feetech_cli.protocol import from_sign_magnitude
from feetech_cli.registers import CONTROL_TABLE
from feetech_cli.registers import format_current


def test_present_current_decodes_bit_15_as_sign():
    # Raw 0x8164 read from a holding servo: magnitude 356, negative.
    sign_bit = CONTROL_TABLE["present_current"].sign_bit
    assert from_sign_magnitude(0x8164, sign_bit) == -356


def test_format_current_converts_known_model():
    assert format_current(-147, 777) == "-147 (-956 mA)"


def test_format_current_labels_phase_current():
    assert format_current(-147, 4618) == "-147 (-956 mA phase)"


def test_format_current_estimates_supply_from_pwm_duty():
    # 3289 mA phase at 30 % duty draws roughly 987 mA from the supply.
    expected = "-506 (-3289 mA phase, ~987 mA supply est.)"
    assert format_current(-506, 4618, load=-300) == expected


def test_format_current_leaves_unknown_model_unconverted():
    assert format_current(-147, 1234) == "-147 (mA scale unknown for model 1234)"
