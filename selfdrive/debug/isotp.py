from enum import IntEnum
from typing import Optional, Set


def hexify(data: bytes) -> str:
  return ''.join(f"\\x{b:02x}" for b in data)


class FrameType(IntEnum):
  SINGLE = 0
  FIRST = 1
  CONSECUTIVE = 2
  FLOW_CONTROL = 3


PACKET_REMAINING = 0


class ISOTPException(Exception):
  pass


class ISOTPFrame():
  type: FrameType
  size: int
  data: bytes
  src: Optional[int]

  def __eq__(self, __o: object) -> bool:
    return isinstance(__o, ISOTPFrame) and self.type == __o.type and self.data == __o.data


# single message (0x00, size (0..7), data...)
class SingleISOTPFrame(ISOTPFrame):
  def __init__(self, dat: bytes, src: Optional[int] = None):
    self.type = FrameType.SINGLE
    self.size = dat[0] & 0x0F
    self.data = dat[1:1 + self.size]
    self.src = src

    assert self.size > 0
    assert len(self.data) == self.size

  def __str__(self) -> str:
    return f"ISOTPFrame.Single(size={self.size}, data={hexify(self.data)})"


# first message of a multi-message (0x01, size (8..4096), data...)
class FirstISOTPFrame(ISOTPFrame):
  def __init__(self, dat: bytes, src: Optional[int] = None):
    self.type = FrameType.FIRST
    self.size = (dat[0] & 0x0F) << 8 | dat[1]
    self.data = dat[2:8]
    self.src = src

    # assert len(self.data) == 6

  def __str__(self) -> str:
    return f"ISOTPFrame.First(size={self.size}, data={hexify(self.data)})"


# consecutive message of a multi-message (0x02, index (0..15), data...)
class ConsecutiveISOTPFrame(ISOTPFrame):
  def __init__(self, dat: bytes, src: Optional[int] = None):
    self.type = FrameType.CONSECUTIVE
    self.index = dat[0] & 0x0F
    self.data = dat[1:]
    self.src = src

    # assert length > 0
    # assert len(self.data) == length, f"{len(self.data)} != {length}"

  def __str__(self) -> str:
    return f"ISOTPFrame.Consecutive(index={self.index}, data={hexify(self.data)})"


# flow control message (0x03, flag (0, 1, 2), block size, separation time)
class FlowISOTPFrame(ISOTPFrame):
  def __init__(self, dat: bytes, src: Optional[int] = None):
    self.type = FrameType.FLOW_CONTROL
    self.flag = dat[0] & 0x0F
    self.block_size = dat[1]
    self.separation_time = dat[2]
    self.src = src

  def __str__(self) -> str:
    return f"ISOTPFrame.Flow(flag={self.flag}, block_size={self.block_size}, separation_time={self.separation_time})"


def parse_isotp_frame(dat: bytes, src: Optional[int] = None) -> Optional[ISOTPFrame]:
  type = FrameType(dat[0] >> 4)
  if type == FrameType.SINGLE:
    return SingleISOTPFrame(dat, src=src)
  elif type == FrameType.FIRST:
    return FirstISOTPFrame(dat, src=src)
  elif type == FrameType.CONSECUTIVE:
    return ConsecutiveISOTPFrame(dat, src=src)
  elif type == FrameType.FLOW_CONTROL:
    return FlowISOTPFrame(dat, src=src)
  raise ISOTPException(f"Unknown frame type: {type}")


class ISOTPPacket():
  def __init__(self, address: int, frame: ISOTPFrame = None, data: bytes = None):
    if frame is not None:
      assert data is None

      if not (frame.type in (FrameType.SINGLE, FrameType.FIRST)):
        raise ISOTPException(f"Cannot start a packet with a {frame.type.name} frame")

      self.address = address
      self._frames = [frame]
      self._size = frame.size
      self.data = bytearray(frame.data)
      self._last_index = 0
      self._srcs: Set[int] = set()
      if frame.src is not None:
        self._srcs.add(frame.src)

      assert frame.type == FrameType.FIRST or len(self.data) == self._size
    else:
      assert data is not None

      self.address = address
      self._size = len(data)
      self.data = data

  def append(self, frame: ISOTPFrame) -> None:
    if frame.type in (FrameType.SINGLE, FrameType.FIRST):
      if frame.type == FrameType.FIRST and frame.size == self._frames[0].size and frame.data == self._frames[0].data:
        # duplicate first frame, ignore
        print("Ignoring duplicate first frame")
        return

      raise ISOTPException(f"Invalid frame type: Tried to append a {frame.type.name} frame to a multi-frame packet")

    self._frames.append(frame)
    if frame.src is not None:
      self._srcs.add(frame.src)

    if frame.type == FrameType.FLOW_CONTROL:
      return

    if self.complete:
      raise ISOTPException("Packet already complete")

    # add consecutive frame to packet data
    expected_index = (self._last_index + 1) % 16
    if frame.index != expected_index:
      if frame.index == self._last_index and frame.data == self._frames[-1].data:
        # duplicate frame, ignore
        print("Ignoring duplicate consecutive frame")
        return

      raise ISOTPException(f"Invalid consecutive frame index (index={frame.index}, expected={expected_index})")

    # TODO: trim data to desired size
    self._last_index = frame.index
    self.data.extend(frame.data)
    if len(self.data) > self._size:
      raise ISOTPException(f"Packet data too large (size={len(self.data)}, expected={self._size})")

  @property
  def length(self) -> int:
    return len(self.data)

  @property
  def complete(self) -> bool:
    if self._frames[0].type == FrameType.SINGLE:
      return True
    # TODO: trim data to desired size
    return self.length >= self._size

  def __str__(self) -> str:
    return f"ISOTPPacket(address={self.address:#04x}, data={hexify(self.data)})"
