"""Command line tools for FEETECH STS/SMS/SCS serial bus servos."""

from importlib.metadata import version

from feetech_cli.controller import FeetechServoController
from feetech_cli.protocol import FeetechChecksumError
from feetech_cli.protocol import FeetechError
from feetech_cli.protocol import FeetechStatusError
from feetech_cli.protocol import FeetechTimeoutError
from feetech_cli.protocol import PacketHandler

# Read from the installed metadata, so pyproject.toml is the only place the
# version is written (bump-version.yml edits nothing else).
__version__ = version("feetech-cli")

__all__ = [
    "FeetechServoController",
    "PacketHandler",
    "FeetechError",
    "FeetechTimeoutError",
    "FeetechChecksumError",
    "FeetechStatusError",
]
