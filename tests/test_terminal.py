"""Drive the interactive screen through a real pty.

These are the only tests that exercise ``readchar`` and the terminal escape
handling for real. They caught a dropped key press bug that every in-process
test missed: the key listener used to keep a single key, so a held down arrow
key lost most of its repeats.
"""

import os
import re
import select
import subprocess
import sys
import time

import pytest

pty = pytest.importorskip("pty")

pytestmark = pytest.mark.skipif(
    os.name != "posix", reason="pty is POSIX only"
)

#: Cursor movement and erase codes, dropped so assertions see plain text.
CURSOR_CODES = re.compile(r"\x1b\[(?!\d+m)[0-9;]*[A-Za-z]")

#: Every escape code, including colours.
ALL_CODES = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

#: How the highlighted id candidate is painted.
GREEN = "\x1b[32m"
RESET = "\x1b[0m"

#: How long to wait for the screen to show something before giving up.
TIMEOUT = 15.0


class Screen:
    """A ``feetech --simulate`` process attached to a pty.

    Parameters
    ----------
    args : list of str, optional
        Extra command line arguments.
    """

    def __init__(self, args=()):
        self.master, slave = pty.openpty()
        environment = dict(os.environ, TERM="xterm", PYTHONUNBUFFERED="1")
        self.process = subprocess.Popen(
            [sys.executable, "-m", "feetech_cli.cli", "--simulate"] + list(args),
            stdin=slave, stdout=slave, stderr=slave,
            close_fds=True, env=environment,
        )
        os.close(slave)
        self.buffer = b""
        self.closed = False

    def pump(self, seconds):
        """Collect output for a while.

        Parameters
        ----------
        seconds : float
            How long to read for.
        """
        end = time.time() + seconds
        while time.time() < end:
            ready, _, _ = select.select([self.master], [], [], 0.05)
            if ready:
                try:
                    self.buffer += os.read(self.master, 65536)
                except OSError:
                    return

    @property
    def text(self):
        """str: Everything painted so far, as plain text."""
        return ALL_CODES.sub("", self.buffer.decode("utf-8", "replace"))

    @property
    def coloured(self):
        """str: Everything painted so far, keeping the colour codes."""
        return CURSOR_CODES.sub("", self.buffer.decode("utf-8", "replace"))

    def highlighted(self):
        """Return the id candidate currently painted green.

        Returns
        -------
        str or None
            The highlighted text, or ``None`` when nothing is highlighted.
        """
        match = re.search(
            re.escape(GREEN) + r"(\d+)" + re.escape(RESET), self.coloured
        )
        if match is None:
            return None
        return match.group(1)

    def line(self, needle):
        """Return the most recently painted line containing ``needle``.

        Parameters
        ----------
        needle : str
            Substring to look for.

        Returns
        -------
        str
            The matching line, or an empty string when there is none.
        """
        hits = [line for line in self.text.splitlines() if needle in line]
        return hits[-1] if hits else ""

    def row(self, label):
        """Return the most recently painted status row for ``label``.

        Rows are prefixed with the cursor marker or with spaces. Matching on
        that keeps the helper from picking up the hint line at the bottom,
        which mentions the selected row's name too.

        Parameters
        ----------
        label : str
            Row label, without the trailing colon.

        Returns
        -------
        str
            The matching line, or an empty string when there is none.
        """
        pattern = re.compile(r"^(?:>> |\s+)" + re.escape(label) + r":")
        hits = [
            line for line in self.text.splitlines() if pattern.match(line)
        ]
        return hits[-1] if hits else ""

    def wait_for_row(self, label, timeout=TIMEOUT):
        """Block until a status row for ``label`` is painted.

        Parameters
        ----------
        label : str
            Row label, without the trailing colon.
        timeout : float, optional
            How long to wait.

        Returns
        -------
        str
            The matching line.
        """
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.1)
            if self.row(label):
                return self.row(label)
        raise AssertionError(
            f"row {label!r} never appeared. Screen was:\n{self.text[-2000:]}"
        )

    def wait_for(self, needle, timeout=TIMEOUT):
        """Block until ``needle`` shows up on screen.

        Parameters
        ----------
        needle : str
            Substring to wait for.
        timeout : float, optional
            How long to wait.

        Returns
        -------
        str
            The line that matched.
        """
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.1)
            if self.line(needle):
                return self.line(needle)
        raise AssertionError(
            f"{needle!r} never appeared. Screen was:\n{self.text[-2000:]}"
        )

    def send(self, keys, settle=0.4):
        """Type into the screen.

        Parameters
        ----------
        keys : str
            Characters or escape sequences to send.
        settle : float, optional
            How long to let the screen react afterwards.
        """
        os.write(self.master, keys.encode())
        self.pump(settle)

    def forget(self):
        """Drop what has been painted so far, so assertions see fresh output."""
        self.buffer = b""

    def close(self):
        """Quit the screen and wait for the process to exit.

        Returns
        -------
        int
            The process exit status.
        """
        try:
            self.send("q", 0.3)
            return self.process.wait(timeout=10)
        except (OSError, subprocess.TimeoutExpired):
            self.process.kill()
            return self.process.wait(timeout=5)
        finally:
            self.release()

    def release(self):
        """Close the pty, at most once."""
        if not self.closed:
            self.closed = True
            os.close(self.master)


@pytest.fixture
def screen():
    """Start the interactive screen on a simulated bus.

    Returns
    -------
    Screen
        The running screen.
    """
    running = Screen()
    try:
        running.wait_for_row("Servo ID")
        yield running
    finally:
        if running.process.poll() is None:
            running.close()
        running.release()


RIGHT = "\x1b[C"
LEFT = "\x1b[D"
UP = "\x1b[A"
DOWN = "\x1b[B"
ENTER = "\n"


def test_the_screen_paints_on_start(screen):
    assert "Servos: 1 2 3" in screen.line("Servos:")


def test_a_simulated_bus_says_so_on_every_frame(screen):
    """The startup banner is wiped by the first repaint, so the header carries
    the warning instead. Otherwise a simulation is indistinguishable from real
    hardware once the screen is up."""
    assert "SIMULATED SERVOS, NO HARDWARE" in screen.text
    screen.forget()
    screen.send(DOWN, 0.6)
    screen.wait_for("Now on")
    assert "SIMULATED SERVOS, NO HARDWARE" in screen.text
    assert "FEETECH Servo Status" not in screen.text


def test_right_arrow_moves_the_id_highlight(screen):
    # The highlighted candidate is the one wrapped in the green escape code.
    assert screen.highlighted() == "1"
    screen.forget()
    screen.send(RIGHT)
    screen.wait_for_row("Servo ID")
    assert screen.highlighted() == "2"


def test_every_repeat_of_a_held_arrow_key_counts(screen):
    """Four quick presses must move four steps, not one.

    The key listener used to keep only the most recent key press, so a burst
    arriving between two frames collapsed into a single step.
    """
    screen.forget()
    screen.send(RIGHT * 4, 1.0)
    screen.wait_for_row("Servo ID")
    assert screen.highlighted() == "5"


def test_enter_writes_the_highlighted_id(screen):
    screen.send(RIGHT * 4, 1.0)
    screen.forget()
    screen.send(ENTER, 1.5)
    screen.wait_for("is now id")
    assert "Servo 1 is now id 5" in screen.text
    assert "Servos: 2 3 5" in screen.wait_for("Servos:")
    assert "Servo ID: 5" in screen.row("Servo ID")


def test_arrows_move_the_servo_even_from_limp(screen):
    """A position command energises the servo, so the arrows always move it.

    The screen must not suggest otherwise: reading "torque is off, it will not
    move" here would be wrong, and would read as safe when it is not.
    """
    assert "Disabled" in screen.row("Torque")
    assert "arrows will energise the servo" in screen.row("Goal Position")
    screen.forget()
    screen.send(DOWN, 0.3)
    screen.send(RIGHT * 3, 1.2)
    # Three steps of about five degrees each, from the centre.
    assert "2216" in screen.wait_for_row("Goal Position")
    assert "2216" in screen.wait_for_row("Present Position")
    assert "Enabled" in screen.row("Torque")
    assert "energised the servo" in screen.text


def test_t_frees_the_joint_again(screen):
    """After the arrows energise the servo, 't' must make it back drivable."""
    screen.send(DOWN, 0.3)
    screen.send(RIGHT, 1.2)
    assert "Enabled" in screen.wait_for_row("Torque")
    screen.forget()
    screen.send("t", 1.2)
    assert "Disabled" in screen.wait_for_row("Torque")
    assert "free to move by hand" in screen.text or "freely" in screen.text


def test_enabling_torque_does_not_snap_to_a_stale_goal(screen):
    """'t' holds where the horn is, not the last goal commanded."""
    screen.send(DOWN, 0.3)
    screen.send(RIGHT * 3, 1.2)
    screen.send("t", 1.0)          # back to limp
    screen.forget()
    screen.send("t", 1.2)          # energise again
    assert "Enabled" in screen.wait_for_row("Torque")
    assert "holding the current position" in screen.text


def test_arrows_on_the_id_row_do_not_move_the_servo(screen):
    """The arrows mean different things per row, and the screen must say so.

    On startup the cursor sits on 'Servo ID', where Left/Right pick an id to
    write rather than move the horn. Without a per row hint this reads as a
    servo that refuses to move even with torque on.
    """
    screen.send("t", 1.0)
    assert "Enabled" in screen.wait_for_row("Torque")
    before = screen.row("Goal Position")
    screen.forget()
    screen.send(RIGHT * 3, 1.0)
    screen.wait_for_row("Servo ID")
    assert screen.row("Goal Position") == before
    # The hint has to name the selected row and what the arrows do there.
    assert "Now on Servo ID" in screen.text
    assert "Left/Right pick the id" in screen.text


def test_the_hint_follows_the_cursor(screen):
    assert "Left/Right pick the id" in screen.text
    screen.forget()
    screen.send(DOWN, 0.6)
    screen.wait_for("Now on")
    assert "Now on Goal Position" in screen.text
    assert "move the servo by about 5 degrees" in screen.text


def test_torque_key_toggles_on_screen(screen):
    assert "Disabled" in screen.row("Torque")
    screen.forget()
    screen.send("t", 0.8)
    assert "Enabled" in screen.wait_for_row("Torque")


def test_next_servo_key_switches_target(screen):
    screen.forget()
    screen.send("n", 0.8)
    screen.wait_for_row("Servo ID")
    assert "Servo ID: 2" in screen.row("Servo ID")


def test_zero_key_asks_then_relabels_the_pose(screen):
    """The '0' key must confirm before it writes EEPROM, then re-label."""
    screen.send(DOWN, 0.3)
    screen.send(RIGHT * 2, 1.2)
    moved_to = screen.wait_for_row("Present Position")
    assert "2048" not in moved_to
    screen.forget()
    screen.send("0", 0.8)
    assert "Press 'y' to confirm" in screen.wait_for("Press 'y' to confirm")
    screen.forget()
    screen.send("y", 1.5)
    screen.wait_for("This pose is now")
    assert "2048" in screen.row("Present Position")
    assert "This pose is now 2048" in screen.text


def test_zero_key_can_be_cancelled(screen):
    screen.forget()
    screen.send("0", 0.8)
    screen.wait_for("Press 'y' to confirm")
    screen.forget()
    screen.send("x", 1.0)
    screen.wait_for("Cancelled")
    assert "2048" in screen.row("Present Position")


def test_quit_key_exits_cleanly(screen):
    assert screen.close() == 0
