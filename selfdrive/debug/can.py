def hexify(data: bytes) -> str:
  return ''.join(f"\\x{b:02x}" for b in data)


class CANFrame():
  def __init__(self, msg=None, src=None, address=None, data=None):
    if msg is not None:
      assert src is None
      assert address is None
      assert data is None
      self.src = int(msg.src)
      self.address = int(msg.address)
      self.data = bytes(msg.dat)
    else:
      # assert src is not None
      assert address is not None
      assert data is not None
      self.src = src
      self.address = address
      self.data = data

  def __str__(self) -> str:
    if self.src is not None:
      return f"CANPacket(src={self.src}, address={self.address:#04x}, data={hexify(self.data)})"
    else:
      return f"CANFrame(address={self.address:#04x}, data={hexify(self.data)})"

  def __csv__(self) -> str:
    # address, data
    return f"{self.address:#04x},{hexify(self.data)}"

  def __hash__(self) -> int:
    hash = 7  # self.address
    for b in self.data:
      hash = hash * 31 + b
    return hash
