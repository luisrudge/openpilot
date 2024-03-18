from openpilot.selfdrive.car import CanBusBase


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def adas(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2


def calculate_checksum(dat: bytearray, init: int) -> int:
  checksum = init
  for i in range(len(dat)):
    # assumes checksum is zeroed out
    checksum += (dat[i] & 0xF) + (dat[i] >> 4)
  return (8 - checksum) & 0xF


def create_lka_msg(packer, apply_steer: float, frame: int, lat_active: bool, max_torque: int):
  values = {
    # 'LOW_SPEED': 0,
    'TORQUE': max_torque if lat_active else 0,
    # 'RIGHT_DEPART': 0,
    # 'LEFT_DEPART': 0,
    # 'STATUS': 0,
    'ACTIVE': int(lat_active),
    # 'RAMP': 0,
    'ANGLE': apply_steer,  # (-90, 90)
    'COUNTER': frame % 0x10,
    'CHECKSUM': 0,
  }

  # calculate checksum
  dat = packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)[2]
  values['CHECKSUM'] = calculate_checksum(dat, 0xD)

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
