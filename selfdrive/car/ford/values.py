from collections import namedtuple
from typing import Dict, List, Union

from cereal import car
from selfdrive.car import dbc_dict
from selfdrive.car.docs_definitions import CarInfo

Ecu = car.CarParams.Ecu

AngleRateLimit = namedtuple('AngleRateLimit', ['speed_bp', 'angle_v'])


class CarControllerParams:
  # Messages: Lane_Assist_Data1, LateralMotionControl
  STEER_STEP = 5
  # Message: ACCDATA
  ACC_CONTROL_STEP = 2
  # Message: IPMA_Data
  LKAS_UI_STEP = 100
  # Message: ACCDATA_3
  ACC_UI_STEP = 5

  CURVATURE_MAX = 0.02  # Max curvature for steering command, m^-1
  STEER_DRIVER_ALLOWANCE = 1.0  # Driver intervention threshold, Nm

  # Curvature rate limits
  # TODO: unify field names used by curvature and angle control cars
  # ~2 m/s^3 up, ~-3 m/s^3 down
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[5, 15, 25], angle_v=[0.004, 0.00044, 0.00016])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[5, 15, 25], angle_v=[0.006, 0.00066, 0.00024])

  ACCEL_MAX = 2.0     # 2.0 m/s^2 max acceleration
  ACCEL_MIN = -3.5    # 3.5 m/s^2 max deceleration


class CANBUS:
  main = 0
  radar = 1
  camera = 2


class CAR:
  ESCAPE_MK4 = "FORD ESCAPE 4TH GEN"
  FOCUS_MK4 = "FORD FOCUS 4TH GEN"
  MAVERICK_MK1 = "FORD MAVERICK 1ST GEN"


CAR_INFO: Dict[str, Union[CarInfo, List[CarInfo]]] = {
}


FW_VERSIONS = {
  CAR.ESCAPE_MK4: {
    (Ecu.eps, 0x730, None): [
      b'LX6C-14D003-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-14D003-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.esp, 0x760, None): [
      b'LX6C-2D053-NS\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-2D053-NY\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-2D053-SA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'LB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'LJ6T-14F397-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LJ6T-14F397-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'LX6A-14C204-BJV\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6A-14C204-ESG\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'MX6A-14C204-BEF\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NX6A-14C204-BLE\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.shiftByWire, 0x732, None): [
      b'LX6P-14G395-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6P-14G395-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PZ1P-14G395-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FOCUS_MK4: {
    (Ecu.eps, 0x730, None): [
      b'JX6C-14D003-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.esp, 0x760, None): [
      b'JX61-2D053-CJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'JX7T-14D049-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'JX7T-14F397-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'JX6A-14C204-BPL\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.shiftByWire, 0x732, None): [
    ],
  },
  CAR.MAVERICK_MK1: {
    (Ecu.eps, 0x730, None): [
      b'NZ6C-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.esp, 0x760, None): [
      b'NZ6C-2D053-AG\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'NZ6T-14D049-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'NZ6T-14F397-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.engine, 0x7E0, None): [
      b'NZ6A-14C204-AAA\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NZ6A-14C204-PA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NZ6A-14C204-ZA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.shiftByWire, 0x732, None): [
      b'NZ6P-14G395-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}


DBC = {
  CAR.ESCAPE_MK4: dbc_dict('ford_lincoln_base_pt', None),
  CAR.FOCUS_MK4: dbc_dict('ford_lincoln_base_pt', None),
  CAR.MAVERICK_MK1: dbc_dict('ford_lincoln_base_pt', None),
}
