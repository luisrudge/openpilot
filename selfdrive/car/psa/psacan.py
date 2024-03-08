def create_lka_msg(packer, apply_steer: float, frame: int, lat_active: bool, max_torque: int):
  values = {
    'COUNTER': frame % 0x10,
    'CHECKSUM': 0,  # TODO: find checksum
    # 'LOW_SPEED': 0,
    'TORQUE': max_torque if lat_active else 0,
    # 'RIGHT_DEPART': 0,
    # 'LEFT_DEPART': 0,
    # 'STATUS': 0,
    'ACTIVE': int(lat_active),
    # 'RAMP': 0,
    'ANGLE': apply_steer,  # (-90, 90)
  }
  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)
