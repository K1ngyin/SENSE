"""Parser and data model for the STM32 chassis feedback frame."""

from dataclasses import dataclass
from typing import List


FRAME_LEN = 24
FRAME_HEAD = 0x7B
FRAME_TAIL = 0x7D


@dataclass(frozen=True)
class FeedbackFrame:
    motor_flag: int
    vx_m_s: float
    vy_m_s: float
    wz_rad_s: float
    accel_m_s2: tuple
    gyro_rad_s: tuple
    battery_v: float


def _signed_int16_be(frame: bytes, offset: int) -> int:
    value = (frame[offset] << 8) | frame[offset + 1]
    return value - 0x10000 if value & 0x8000 else value


def _unsigned_int16_be(frame: bytes, offset: int) -> int:
    return (frame[offset] << 8) | frame[offset + 1]


def checksum_bcc(frame: bytes) -> int:
    """Return XOR of bytes 0 through 21, matching serial_monitor.py."""
    value = 0
    for byte in frame[:22]:
        value ^= byte
    return value


def decode_frame(frame: bytes) -> FeedbackFrame:
    """Validate and decode one complete 24-byte feedback frame."""
    if len(frame) != FRAME_LEN:
        raise ValueError("frame length is not 24 bytes")
    if frame[0] != FRAME_HEAD or frame[23] != FRAME_TAIL:
        raise ValueError("frame header or tail is invalid")
    if checksum_bcc(frame) != frame[22]:
        raise ValueError("BCC checksum failed")

    return FeedbackFrame(
        motor_flag=frame[1],
        vx_m_s=_signed_int16_be(frame, 2) / 1000.0,
        vy_m_s=_signed_int16_be(frame, 4) / 1000.0,
        wz_rad_s=_signed_int16_be(frame, 6) / 1000.0,
        accel_m_s2=tuple(_signed_int16_be(frame, i) / 1672.0 for i in (8, 10, 12)),
        gyro_rad_s=tuple(_signed_int16_be(frame, i) / 3753.0 for i in (14, 16, 18)),
        battery_v=_unsigned_int16_be(frame, 20) / 1000.0,
    )


class FrameParser:
    """Incremental parser that re-synchronizes after noise or invalid frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bytes_discarded = 0
        self.bad_tail_count = 0
        self.bad_checksum_count = 0
        self.valid_frame_count = 0

    def feed(self, data: bytes) -> List[FeedbackFrame]:
        self._buffer.extend(data)
        frames: List[FeedbackFrame] = []

        while self._buffer:
            try:
                head_index = self._buffer.index(FRAME_HEAD)
            except ValueError:
                self.bytes_discarded += len(self._buffer)
                self._buffer.clear()
                break

            if head_index:
                self.bytes_discarded += head_index
                del self._buffer[:head_index]

            if len(self._buffer) < FRAME_LEN:
                break

            if self._buffer[FRAME_LEN - 1] != FRAME_TAIL:
                self.bad_tail_count += 1
                del self._buffer[0]
                continue

            candidate = bytes(self._buffer[:FRAME_LEN])
            if checksum_bcc(candidate) != candidate[22]:
                self.bad_checksum_count += 1
                del self._buffer[0]
                continue

            # Header, tail, and checksum were checked above; keep decoding in one place.
            frames.append(decode_frame(candidate))
            self.valid_frame_count += 1
            del self._buffer[:FRAME_LEN]

        return frames
