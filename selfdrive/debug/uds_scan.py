#!/usr/bin/env python3
# pylint: skip-file
# flake8: noqa
# type: ignore

import multiprocessing
from pathlib import Path
import sys
import time
from typing import List, Optional

from tools.lib.logreader import LogReader
from tools.lib.route import Route

from selfdrive.debug.can import CANFrame
from selfdrive.debug.isotp import FrameType, ISOTPPacket, ISOTPFrame, parse_isotp_frame
from selfdrive.debug.uds import UDSPacket, parse_uds_packet, SecurityAccessService


def hexify(data: bytes) -> str:
  return ''.join(f"\\x{b:02x}" for b in data)


def load_segment(segment_name: Optional[str]) -> List[LogReader]:
  print(f"Loading {segment_name}")
  if segment_name is None:
    return []

  try:
    return list(LogReader(segment_name))
  except ValueError as e:
    print(f"Error parsing {segment_name}: {e}")
    return []


def write_can_to_csv(can_frames: List[CANFrame], csv_file: str) -> None:
  with open(csv_file, 'w') as f:
    f.write("address,data\n")
    for can_frame in can_frames:
      f.write(f"{can_frame.address:#04x},{bytes.hex(can_frame.data)}\n")


def read_csv_to_can(csv_file: str) -> List[CANFrame]:
  can_frames: List[CANFrame] = []
  with open(csv_file, 'r') as f:
    header = True
    for line in f:
      if header:
        header = False
        continue
      # 0x000,\x00\x00\x00\x00\x00\x00\x00\x00
      address, dat = line.split(',')
      can_frames.append(CANFrame(address=int(address, 16), data=bytes.fromhex(dat)))
  return can_frames


# def write_isotp_to_csv(isotp_packets: List[ISOTPPacket], csv_file: str) -> None:
#   with open(csv_file, 'w') as f:
#     f.write("address,dat\n")
#     for isotp_packet in isotp_packets:
#       f.write(f"{isotp_packet.address:#04x},{isotp_packet.data.hex()}\n")


# def read_csv_to_isotp(csv_file: str) -> List[ISOTPPacket]:
#   isotp_packets: List[ISOTPPacket] = []
#   with open(csv_file, 'r') as f:
#     header = True
#     for line in f:
#       if header:
#         header = False
#         continue
#       # 0x000,\x00\x00\x00\x00\x00\x00\x00\x00
#       address, dat = line.split(',')
#       isotp_packets.append(ISOTPPacket(address=int(address, 16), data=bytes.fromhex(dat)))
#   return isotp_packets


if __name__ == "__main__":
  FILTER_CAN_ECU = True
  ECU_ADDR = 0x730
  RX_OFFSET = 0x8

  ROUTES = [
    # "86d00e12925f4df7|2022-06-25--12-10-57",
    # "86d00e12925f4df7|2022-06-25--12-18-27",
    # "86d00e12925f4df7|2022-06-25--12-29-00",

    # "86d00e12925f4df7|2022-07-17--22-17-04",  # first attempt, didn't work (only tester present queries)
    # "86d00e12925f4df7|2022-07-17--22-17-40",
    # "86d00e12925f4df7|2022-07-17--22-22-15",
    # "86d00e12925f4df7|2022-07-17--22-24-45",
    # "86d00e12925f4df7|2022-07-17--22-25-00",
    # "86d00e12925f4df7|2022-07-17--22-27-06",
    # "86d00e12925f4df7|2022-07-17--22-28-57",
    # "86d00e12925f4df7|2022-07-17--22-30-08",  # successful fingerprint (all queries?)
    # "86d00e12925f4df7|2022-07-17--22-31-17",
    # "86d00e12925f4df7|2022-07-17--22-33-18",
    # "86d00e12925f4df7|2022-07-17--22-34-11",
    # "86d00e12925f4df7|2022-07-17--22-34-50",
    # "86d00e12925f4df7|2022-07-17--22-35-29",
    # "86d00e12925f4df7|2022-07-17--22-36-26",
    # "86d00e12925f4df7|2022-07-17--22-37-09",  # last attempt, didn't work? (all queries?)

    "86d00e12925f4df7|2022-07-18--20-24-15",
  ]

  CACHE_LOGS = False
  # CACHE_ISOTP = False

  can_frames: List[CANFrame] = []
  st = time.monotonic()
  for name in ROUTES:
    print(f"Loading route {name}")

    # Load CAN frames for the route from file
    if Path(f"{name}-all.csv").is_file() and CACHE_LOGS:
      route_can_frames = read_csv_to_can(f"{name}-all.csv")
      can_frames += route_can_frames

    # Download and process route logs
    else:
      with multiprocessing.Pool(24) as pool:
        paths = Route(name).log_paths()
        if None in paths:
          print(f"Error loading route {name} - missing logs")
          continue

        msgs = []
        for d in pool.map(load_segment, paths):
          msgs += d

        # Parse CAN data
        route_can_frames: List[CANFrame] = []
        for m in msgs:
          if m.which() == 'can':
            for can in m.can:
              route_can_frames.append(CANFrame(can))

        # Write to CSV file
        write_can_to_csv(route_can_frames, f"{name}-all.csv")

        # We only care about messages on the OBD port bus
        # route_can_frames = [f for f in route_can_frames if f.src == 1]

        can_frames += route_can_frames


  # Filter out messages from other ECUs
  if FILTER_CAN_ECU:
    can_frames = [c for c in can_frames if c.address >= 0x700 and c.address <= 0x7ff]
    # can_frames = [c for c in can_frames if c.address in (0x7DF, 0x7E0, 0x7E8)]
    print(f"Filtering CAN messages: {len(can_frames)} CAN messages")

  # Print address counts
  address_count = {}
  for can in can_frames:
    address_count[can.address] = address_count.get(can.address, 0) + 1

  count_map = {}
  for address, count in address_count.items():
    count_map[count] = count_map.get(count, []) + [f"{address:#04x}"]

  print("Address counts:")
  for count, addresses in sorted(count_map.items(), reverse=True):
    print(f"  {count}: {sorted(addresses)}")
  print()


  # Load ISOTP packets from file
  # if Path(f"{name}-isotp.csv").is_file() and CACHE_ISOTP:
  #   st = time.monotonic()
  #   isotp_packets = read_csv_to_isotp(f"{name}-isotp.csv")
  #   et = time.monotonic()
  #   print(f"Loaded {len(isotp_packets)} ISOTP packets in {et - st:.2f}s")

  # Build ISOTP packets
  isotp_packets: List[ISOTPPacket] = []
  last_isotp_frame: Optional[ISOTPFrame] = None
  building = {}
  DEBUG = False

  for can_frame in can_frames:
    try:
      isotp_frame = parse_isotp_frame(can_frame.data, src=can_frame.src)
    except:
      continue

    # Skip flow control frames
    if isotp_frame.type == FrameType.FLOW_CONTROL:
      continue

    # Skip duplicate frames
    if last_isotp_frame is not None and last_isotp_frame == isotp_frame:
      continue

    address = can_frame.address
    if DEBUG: print(f"Processing {address:#04x} {isotp_frame}")

    # Find existing packet being built for this address
    isotp_packet: Optional[ISOTPPacket] = building.get(address, None)

    # If no packet is being built, start a new one
    if isotp_packet is None:
      try:
        isotp_packet = ISOTPPacket(address, isotp_frame)
        building[address] = isotp_packet
        if DEBUG: print(f"Starting new packet for {address:#04x}")
      except Exception as e:
        print(f"Error creating ISOTPPacket from {isotp_frame}: {e}")

    # Otherwise, add the frame to the existing packet
    else:
      assert not isotp_packet.complete, "Packet already complete"
      assert isotp_packet._frames[0].type == FrameType.FIRST

      # Try adding the frame to the existing packet...
      # We do this even if it's a first/single frame because the ISOTPPacket
      # may resolve the problem itself, for example by discarding it if it is
      # a duplicate.
      try:
        if DEBUG: print(f"Appending {isotp_frame.type.name} frame to packet on {address:#04x}")
        isotp_packet.append(isotp_frame)
      except Exception as e:
        print(f"Error appending {isotp_frame} to packet: {e}")

        # Attempt to resolve the error: maybe we are starting a new packet early?
        if isotp_frame.type in (FrameType.SINGLE, FrameType.FIRST):
          if DEBUG:
            print(f"Oops! Got {isotp_frame.type.name} frame in middle of multi-frame packet on {address:#04x}")
            print(f"\t{isotp_packet}")
            for frame in isotp_packet._frames:
              print(f"\t\t{frame}")
            print(f"\t(Submitting packet when {isotp_packet._size - isotp_packet.length} bytes too short)")

          # Submit the previous unfinished packet
          isotp_packets.append(isotp_packet)

          # Start a new packet
          isotp_packet = ISOTPPacket(address, isotp_frame)
          building[address] = isotp_packet
          if DEBUG: print(f"Starting new packet for {address:#04x}")

    # If the packet is complete, add it to the list and remove it from the current list
    if isotp_packet is not None and isotp_packet.complete:
      if DEBUG: print(f"Finished packet on {address:#04x}")
      isotp_packets.append(isotp_packet)
      building[address] = None

    if DEBUG: print()
  print()

  # Write to CSV file
  # write_isotp_to_csv(isotp_packets, f"{name}-isotp.csv")


  import traceback

  # Parse UDS packets
  uds_packets: List[UDSPacket] = []
  last_uds_packet: UDSPacket = None
  for isotp_packet in isotp_packets:
    # TODO: filter addresses, we only care about some ECUs
    if isotp_packet.address > 0x1000:
      continue

    uds_packet: Optional = None
    try:
      uds_packet = parse_uds_packet(isotp_packet.data, isotp_packet._srcs)
    except Exception as e:
      print(f"Error parsing UDS packet: {e}")
      traceback.print_exc()

    data = hexify(isotp_packet.data)

    sb = f"{isotp_packet.address:#04x} "
    if uds_packet is not None:
      if uds_packet == last_uds_packet:
        continue
      uds_packets.append(uds_packet)
      sb += f" {uds_packet}"
    print(sb)
    last_uds_packet = uds_packet

  print()
