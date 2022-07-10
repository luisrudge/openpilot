#!/usr/bin/env python3

import argparse
import struct
from enum import IntEnum
from panda import Panda
from panda.python.uds import UdsClient, MessageTimeoutError, NegativeResponseError, SESSION_TYPE,\
  DATA_IDENTIFIER_TYPE, ACCESS_TYPE

FORD_EPS_CAN_ADDR = 0x730
RX_OFFSET = 0x40  # or 8..?

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--debug", action="store_true", help="enable ISO-TP/UDS stack debugging output")
  parser.add_argument("action", choices={"show", "enable", "disable"}, help="show or modify current EPS TJA config")
  args = parser.parse_args()

  panda = Panda()
  panda.set_safety_mode(Panda.SAFETY_ELM327)
  bus = 1 if panda.has_obd() else 0
  uds_client = UdsClient(panda, FORD_EPS_CAN_ADDR, FORD_EPS_CAN_ADDR + RX_OFFSET, bus, timeout=0.2, debug=args.debug)

  try:
    uds_client.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
  except MessageTimeoutError:
    print("Timeout opening session with EPS")
    quit()

  uds_data_ids = {}
  for std_id in DATA_IDENTIFIER_TYPE:
    uds_data_ids[std_id.value] = std_id.name

  resp = {}
  for uds_data_id in sorted(uds_data_ids):
    try:
      data = uds_client.read_data_by_identifier(uds_data_id)  # type: ignore
      if data:
        resp[uds_data_id] = data
    except (NegativeResponseError, MessageTimeoutError):
      pass

  print(f"\n\n*** Results for address 0x{FORD_EPS_CAN_ADDR:X} ***\n\n")
  for rid, dat in resp.items():
    print(f"0x{rid:02X} {uds_data_ids[rid]}: {dat}")

  odx_file, current_coding = None, None
  try:
    hw_pn = uds_client.read_data_by_identifier(DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_HARDWARE_NUMBER).decode("utf-8")
    sw_pn = uds_client.read_data_by_identifier(DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_SPARE_PART_NUMBER).decode("utf-8")
    sw_ver = uds_client.read_data_by_identifier(DATA_IDENTIFIER_TYPE.VEHICLE_MANUFACTURER_ECU_SOFTWARE_VERSION_NUMBER).decode("utf-8")
    component = uds_client.read_data_by_identifier(DATA_IDENTIFIER_TYPE.SYSTEM_NAME_OR_ENGINE_TYPE).decode("utf-8")
    odx_file = uds_client.read_data_by_identifier(DATA_IDENTIFIER_TYPE.ODX_FILE).decode("utf-8")
    # current_coding = uds_client.read_data_by_identifier(VOLKSWAGEN_DATA_IDENTIFIER_TYPE.CODING)  # type: ignore
    # coding_text = current_coding.hex()

    print("\nEPS diagnostic data\n")
    print(f"   Part No HW:   {hw_pn}")
    print(f"   Part No SW:   {sw_pn}")
    print(f"   SW Version:   {sw_ver}")
    print(f"   Component:    {component}")
    # print(f"   Coding:       {coding_text}")
    print(f"   ASAM Dataset: {odx_file}")
  except NegativeResponseError:
    print("Error fetching data from EPS")
    quit()
  except MessageTimeoutError:
    print("Timeout fetching data from EPS")
    quit()

  coding_variant, current_coding_array = None, None
  # EV_SteerAssisMQB covers the majority of MQB racks (EPS_MQB_ZFLS)
  # APA racks (MQB_PP_APA) have a different coding layout, which should
  # be easy to support once we identify the specific config bit
  if odx_file == "EV_SteerAssisMQB\x00":
    coding_variant = "ZF"
    current_coding_array = struct.unpack("!4B", current_coding)
    hca_enabled = (current_coding_array[0] & (1 << 4) != 0)
    hca_text = ("DISABLED", "ENABLED")[hca_enabled]
    print(f"   Traffic Jam Assist:  {hca_text}")
  else:
    print("Configuration changes not yet supported on this EPS!")
    quit()

  try:
    params = uds_client.read_data_by_identifier(DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION).decode("utf-8")
    param_version_system_params = params[1:3]
    param_vehicle_type = params[3:5]
    param_index_char_curve = params[5:7]
    param_version_char_values = params[7:9]
    param_version_memory_map = params[9:11]
    print("\nEPS parameterization (per-vehicle calibration) data\n")
    print(f"   Version of system parameters:     {param_version_system_params}")
    print(f"   Vehicle type:                     {param_vehicle_type}")
    print(f"   Index of characteristic curve:    {param_index_char_curve}")
    print(f"   Version of characteristic values: {param_version_char_values}")
    print(f"   Version of memory map:            {param_version_memory_map}")
  except (NegativeResponseError, MessageTimeoutError):
    print("Error fetching parameterization data from EPS!")
    quit()

  if args.action in ["enable", "disable"]:
    print("\nAttempting configuration update")

    assert(coding_variant == "ZF")  # revisit when we have the APA rack coding bit
    # ZF EPS config coding length can be anywhere from 1 to 4 bytes, but the
    # bit we care about is always in the same place in the first byte
    if args.action == "enable":
      new_byte_0 = current_coding_array[0] | (1 << 4)
    else:
      new_byte_0 = current_coding_array[0] & ~(1 << 4)
    new_coding = new_byte_0.to_bytes(1, "little") + current_coding[1:]

    try:
      seed = uds_client.security_access(ACCESS_TYPE_LEVEL_1.REQUEST_SEED)  # type: ignore
      key = struct.unpack("!I", seed)[0] + 28183  # yeah, it's like that
      uds_client.security_access(ACCESS_TYPE_LEVEL_1.SEND_KEY, struct.pack("!I", key))  # type: ignore
    except (NegativeResponseError, MessageTimeoutError):
      print("Security access failed!")
      quit()

    try:
      # Programming date and tester number must be written before making
      # a change, or write to CODING will fail with request sequence error
      # Encoding on tester is unclear, it contains the workshop code in the
      # last two bytes, but not the VZ/importer or tester serial number
      # Can't seem to read it back, but we can read the calibration tester,
      # so fib a little and say that same tester did the programming
      # TODO: encode the actual current date
      prog_date = b'\x22\x02\x08'
      uds_client.write_data_by_identifier(DATA_IDENTIFIER_TYPE.PROGRAMMING_DATE, prog_date)
      tester_num = uds_client.read_data_by_identifier(DATA_IDENTIFIER_TYPE.CALIBRATION_REPAIR_SHOP_CODE_OR_CALIBRATION_EQUIPMENT_SERIAL_NUMBER)
      uds_client.write_data_by_identifier(DATA_IDENTIFIER_TYPE.REPAIR_SHOP_CODE_OR_TESTER_SERIAL_NUMBER, tester_num)
      uds_client.write_data_by_identifier(VOLKSWAGEN_DATA_IDENTIFIER_TYPE.CODING, new_coding)  # type: ignore
    except (NegativeResponseError, MessageTimeoutError):
      print("Writing new configuration failed!")
      quit()

    try:
      # Read back result just to make 100% sure everything worked
      current_coding_text = uds_client.read_data_by_identifier(VOLKSWAGEN_DATA_IDENTIFIER_TYPE.CODING).hex()  # type: ignore
      print(f"   New coding:   {current_coding_text}")
    except (NegativeResponseError, MessageTimeoutError):
      print("Reading back updated coding failed!")
      quit()
    print("EPS configuration successfully updated")
