"""Shared test fixtures."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


@pytest.fixture(autouse=True)
def no_installed_device_files(monkeypatch, tmp_path):
    """Keep the developer's own device files out of every test."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "config"))
    monkeypatch.delenv("FEETECH_CLI_DEVICES", raising=False)
