"""Aliases for the in-package servo simulator, used by the test suite."""

from feetech_cli.simulator import SimulatedBus as FakeBus
from feetech_cli.simulator import SimulatedServo as FakeServo

__all__ = ["FakeBus", "FakeServo"]
