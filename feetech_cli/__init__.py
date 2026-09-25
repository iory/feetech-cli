"""Command line tools for FEETECH STS/SMS/SCS serial bus servos."""

from feetech_cli.controller import FeetechServoController
from feetech_cli.protocol import FeetechChecksumError
from feetech_cli.protocol import FeetechError
from feetech_cli.protocol import FeetechStatusError
from feetech_cli.protocol import FeetechTimeoutError
from feetech_cli.protocol import PacketHandler

__version__ = "0.1.1"

__all__ = [
    "FeetechServoController",
    "PacketHandler",
    "FeetechError",
    "FeetechTimeoutError",
    "FeetechChecksumError",
    "FeetechStatusError",
]
