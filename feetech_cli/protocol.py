"""Packet level implementation of the FEETECH serial bus servo protocol.

FEETECH STS/SMS/SCS servos speak a Dynamixel-1.0 style protocol over a
half duplex TTL bus::

    instruction : 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
    status      : 0xFF 0xFF ID LENGTH ERROR       PARAM... CHECKSUM

``LENGTH`` counts the bytes that follow it, i.e. ``len(params) + 2``.
``CHECKSUM`` is ``~(ID + LENGTH + INSTRUCTION + sum(params)) & 0xFF``.

The STS/SMS series stores multi byte registers little endian while the older
SCS series stores them big endian, which is why the byte order is a property
of :class:`PacketHandler` rather than a module constant.
"""

import time

import serial

BROADCAST_ID = 0xFE
MAX_ID = 0xFC

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_ACTION = 0x05
INST_FACTORY_RESET = 0x06
INST_SYNC_WRITE = 0x83

#: Hardware error bits reported in the ``ERROR`` byte of a status packet.
ERROR_BITS = (
    (0x01, "voltage"),
    (0x02, "angle"),
    (0x04, "overheat"),
    (0x08, "over-element"),
    (0x10, "over-current"),
    (0x20, "reserved-5"),
    (0x40, "overload"),
    (0x80, "reserved-7"),
)


class FeetechError(Exception):
    """Base class for every error raised by this package."""


class FeetechTimeoutError(FeetechError):
    """No (complete) status packet arrived before the timeout expired."""


class FeetechChecksumError(FeetechError):
    """A status packet arrived but its checksum did not match."""


class FeetechStatusError(FeetechError):
    """The servo answered with a non zero hardware error byte.

    Attributes
    ----------
    servo_id : int
        Id of the servo that reported the error.
    error : int
        Raw error byte.
    """

    def __init__(self, servo_id, error):
        self.servo_id = servo_id
        self.error = error
        super().__init__(
            "servo {} reported error 0x{:02X} ({})".format(
                servo_id, error, ", ".join(describe_error(error))
            )
        )


def describe_error(error):
    """Translate a status error byte into human readable flag names.

    Parameters
    ----------
    error : int
        Error byte taken from a status packet.

    Returns
    -------
    list of str
        Names of the flags that are set, or ``["none"]`` when ``error`` is 0.
    """
    names = [name for bit, name in ERROR_BITS if error & bit]
    if not names:
        return ["none"]
    return names


def checksum(payload):
    """Compute the protocol checksum over a packet body.

    Parameters
    ----------
    payload : iterable of int
        Every byte of the packet except the two header bytes and the
        checksum itself, i.e. ``ID, LENGTH, INSTRUCTION, *PARAMS``.

    Returns
    -------
    int
        The checksum byte.
    """
    return (~sum(payload)) & 0xFF


def to_sign_magnitude(value, sign_bit):
    """Encode a signed integer in FEETECH sign-magnitude form.

    Parameters
    ----------
    value : int
        Signed value to encode.
    sign_bit : int
        Index of the bit that carries the sign.

    Returns
    -------
    int
        Encoded unsigned value.

    Raises
    ------
    ValueError
        If the magnitude does not fit below ``sign_bit``. Encoding it anyway
        would silently overwrite the sign bit and change the value.
    """
    magnitude = abs(value)
    if magnitude >= (1 << sign_bit):
        raise ValueError(
            f"value {value} does not fit in {sign_bit} magnitude bits"
        )
    if value < 0:
        return magnitude | (1 << sign_bit)
    return magnitude


def from_sign_magnitude(value, sign_bit):
    """Decode a FEETECH sign-magnitude integer.

    Parameters
    ----------
    value : int
        Raw register value.
    sign_bit : int
        Index of the bit that carries the sign.

    Returns
    -------
    int
        Decoded signed value.
    """
    magnitude = value & ~(1 << sign_bit)
    if value & (1 << sign_bit):
        return -magnitude
    return magnitude


class PacketHandler:
    """Send instruction packets and receive status packets over a serial port.

    Parameters
    ----------
    port : serial.Serial
        An open serial port connected to the servo bus.
    little_endian : bool, optional
        ``True`` for the STS/SMS series (default), ``False`` for the SCS series.
    """

    #: Upper bound on the bytes scanned while hunting for a packet header, so
    #: that a noisy or constantly talking bus cannot spin forever.
    max_sync_bytes = 512

    def __init__(self, port, little_endian=True):
        self.port = port
        self.little_endian = little_endian

    def encode_word(self, value):
        """Split a 16 bit value into two bytes using the bus byte order.

        Parameters
        ----------
        value : int
            Value in the range 0-65535.

        Returns
        -------
        list of int
            The two parameter bytes in transmission order.
        """
        low = value & 0xFF
        high = (value >> 8) & 0xFF
        if self.little_endian:
            return [low, high]
        return [high, low]

    def decode_word(self, data):
        """Join two received bytes into a 16 bit value using the bus byte order.

        Parameters
        ----------
        data : sequence of int
            Exactly two bytes in reception order.

        Returns
        -------
        int
            The decoded value.
        """
        first, second = data[0], data[1]
        if self.little_endian:
            return (second << 8) | first
        return (first << 8) | second

    def encode(self, size, value):
        """Encode a register value into parameter bytes.

        Parameters
        ----------
        size : int
            Register width in bytes, 1 or 2.
        value : int
            Value to encode.

        Returns
        -------
        list of int
            Parameter bytes in transmission order.
        """
        if size == 1:
            return [value & 0xFF]
        if size == 2:
            return self.encode_word(value)
        raise ValueError(f"unsupported register size: {size}")

    def decode(self, size, data):
        """Decode parameter bytes into a register value.

        Parameters
        ----------
        size : int
            Register width in bytes, 1 or 2.
        data : sequence of int
            Received parameter bytes.

        Returns
        -------
        int
            The decoded value.
        """
        if size == 1:
            return data[0]
        if size == 2:
            return self.decode_word(data)
        raise ValueError(f"unsupported register size: {size}")

    def build_packet(self, servo_id, instruction, params=()):
        """Assemble a complete instruction packet.

        Parameters
        ----------
        servo_id : int
            Target servo id, or :data:`BROADCAST_ID`.
        instruction : int
            One of the ``INST_*`` constants.
        params : sequence of int, optional
            Instruction parameters.

        Returns
        -------
        bytes
            The bytes to put on the wire.
        """
        params = list(params)
        body = [servo_id & 0xFF, len(params) + 2, instruction] + params
        return bytes([0xFF, 0xFF] + body + [checksum(body)])

    def _send(self, packet):
        """Flush the receive buffer and transmit one packet.

        Parameters
        ----------
        packet : bytes
            Packet produced by :meth:`build_packet`.
        """
        self.port.reset_input_buffer()
        self.port.write(packet)
        self.port.flush()

    def _read_exact(self, count):
        """Read exactly ``count`` bytes or raise.

        Parameters
        ----------
        count : int
            Number of bytes to read.

        Returns
        -------
        bytes
            The bytes read.

        Raises
        ------
        FeetechTimeoutError
            If fewer than ``count`` bytes arrived before the timeout.
        """
        data = self.port.read(count)
        if len(data) < count:
            raise FeetechTimeoutError(
                f"expected {count} bytes, received {len(data)}"
            )
        return data

    def _read_status_packet(self, sent=None):
        """Receive one status packet.

        Parameters
        ----------
        sent : bytes, optional
            The instruction packet that was just transmitted. Some half duplex
            adapters echo the transmission back onto the receive line; when the
            first packet read is byte for byte identical to ``sent`` it is
            discarded and the next one is read instead.

        Returns
        -------
        tuple of (int, int, bytes)
            Servo id, error byte and the parameter bytes.

        Raises
        ------
        FeetechTimeoutError
            If no complete packet arrived in time.
        FeetechChecksumError
            If the packet checksum did not match.
        """
        for _ in range(2):
            servo_id, error, params, raw = self._read_one_packet()
            if sent is not None and raw == sent:
                # Adapter echoed our own transmission, read the real answer.
                continue
            return servo_id, error, params
        raise FeetechTimeoutError("only the transmit echo was received")

    def _read_one_packet(self):
        """Scan for a header and read a single packet off the wire.

        Returns
        -------
        tuple of (int, int, bytes, bytes)
            Servo id, error byte, parameter bytes and the complete raw packet.
        """
        window = bytearray()
        for _ in range(self.max_sync_bytes):
            byte = self.port.read(1)
            if len(byte) == 0:
                raise FeetechTimeoutError("no packet header received")
            window.append(byte[0])
            if len(window) > 2:
                del window[0]
            if len(window) == 2 and window[0] == 0xFF and window[1] == 0xFF:
                break
        else:
            raise FeetechTimeoutError(
                f"no packet header found within {self.max_sync_bytes} bytes"
            )

        head = self._read_exact(2)
        servo_id, length = head[0], head[1]
        if length < 2:
            raise FeetechChecksumError(f"invalid packet length: {length}")
        body = self._read_exact(length)
        error = body[0]
        params = bytes(body[1:-1])
        expected = checksum([servo_id, length, error] + list(params))
        if body[-1] != expected:
            raise FeetechChecksumError(
                f"checksum mismatch: got 0x{body[-1]:02X}, expected 0x{expected:02X}"
            )
        raw = bytes([0xFF, 0xFF]) + head + body
        return servo_id, error, params, raw

    def txrx(self, servo_id, instruction, params=(), raise_on_error=True):
        """Send an instruction and wait for the matching status packet.

        Parameters
        ----------
        servo_id : int
            Target servo id.
        instruction : int
            One of the ``INST_*`` constants.
        params : sequence of int, optional
            Instruction parameters.
        raise_on_error : bool, optional
            Raise :class:`FeetechStatusError` when the servo reports a non zero
            error byte. Set to ``False`` to inspect the error yourself.

        Returns
        -------
        tuple of (int, bytes)
            The error byte and the returned parameter bytes.
        """
        packet = self.build_packet(servo_id, instruction, params)
        self._send(packet)
        answer_id, error, answer = self._read_status_packet(sent=packet)
        if answer_id != servo_id:
            raise FeetechChecksumError(
                f"answer came from servo {answer_id} but {servo_id} was addressed"
            )
        if error and raise_on_error:
            raise FeetechStatusError(servo_id, error)
        return error, answer

    def txonly(self, servo_id, instruction, params=()):
        """Send an instruction without waiting for a status packet.

        Parameters
        ----------
        servo_id : int
            Target servo id, typically :data:`BROADCAST_ID`.
        instruction : int
            One of the ``INST_*`` constants.
        params : sequence of int, optional
            Instruction parameters.
        """
        self._send(self.build_packet(servo_id, instruction, params))

    def ping(self, servo_id):
        """Check whether a servo answers on the bus.

        Parameters
        ----------
        servo_id : int
            Servo id to probe.

        Returns
        -------
        bool
            ``True`` if the servo answered with a valid packet.
        """
        try:
            self.txrx(servo_id, INST_PING, raise_on_error=False)
        except (FeetechError, serial.SerialException):
            return False
        return True

    def read(self, servo_id, address, size):
        """Read a register.

        Parameters
        ----------
        servo_id : int
            Servo id.
        address : int
            Register address.
        size : int
            Number of bytes to read.

        Returns
        -------
        bytes
            The register contents.
        """
        _, answer = self.txrx(servo_id, INST_READ, [address & 0xFF, size & 0xFF])
        if len(answer) != size:
            raise FeetechChecksumError(
                f"expected {size} bytes from address {address}, received {len(answer)}"
            )
        return answer

    def write(self, servo_id, address, data, expect_status=True):
        """Write raw bytes to a register.

        Parameters
        ----------
        servo_id : int
            Servo id, or :data:`BROADCAST_ID` to address every servo.
        address : int
            Register address.
        data : sequence of int
            Bytes to write.
        expect_status : bool, optional
            Wait for the status packet. Must be ``False`` for broadcast writes
            and for servos whose ``Response_Status_Level`` is 0, because those
            never answer a write.
        """
        params = [address & 0xFF] + [byte & 0xFF for byte in data]
        if servo_id == BROADCAST_ID or not expect_status:
            self.txonly(servo_id, INST_WRITE, params)
            # Give the servo time to consume the packet before the next one.
            time.sleep(0.002)
            return
        self.txrx(servo_id, INST_WRITE, params)

    def sync_write(self, address, entries):
        """Write the same registers on many servos in one packet.

        A sync write is a broadcast, so no servo answers and none can report a
        failure. It exists because addressing servos one at a time costs a
        round trip each, which does not fit a 50 Hz control loop.

        Parameters
        ----------
        address : int
            First register address, the same for every servo.
        entries : sequence of tuple
            ``(servo_id, data)`` pairs. Every ``data`` must be the same length.

        Raises
        ------
        ValueError
            If the entries disagree on how many bytes they write.
        """
        entries = list(entries)
        if not entries:
            return
        data_length = len(entries[0][1])
        if any(len(data) != data_length for _, data in entries):
            raise ValueError("every sync write entry must be the same length")
        params = [address & 0xFF, data_length]
        for servo_id, data in entries:
            params.append(servo_id & 0xFF)
            params.extend(byte & 0xFF for byte in data)
        self.txonly(BROADCAST_ID, INST_SYNC_WRITE, params)

    def factory_reset(self, servo_id, expect_status=True):
        """Restore the factory defaults of a servo.

        Parameters
        ----------
        servo_id : int
            Servo id.
        expect_status : bool, optional
            Wait for the status packet.
        """
        if not expect_status:
            self.txonly(servo_id, INST_FACTORY_RESET)
            return
        self.txrx(servo_id, INST_FACTORY_RESET)
