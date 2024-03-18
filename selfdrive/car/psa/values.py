from dataclasses import dataclass, field

from cereal import car
from openpilot.selfdrive.car import CarSpecs, dbc_dict, PlatformConfig, Platforms
from openpilot.selfdrive.car.docs_definitions import CarDocs, CarHarness, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig

Ecu = car.CarParams.Ecu


class CarControllerParams:
  STEER_STEP = 5  # LANE_KEEP_ASSIST, 20Hz

  STEER_MAX = 90.0  # Max angle for LKA
  LKAS_MAX_TORQUE = 100  # TODO: verify
  STEER_THRESHOLD = 50  # TODO: verify

  STEER_DRIVER_ALLOWANCE = 2  # Driver intervention threshold

  def __init__(self, CP):
    pass


@dataclass
class PSACarDocs(CarDocs):
  package: str = "Adaptive Cruise Control"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.psa_a]))


class CAR(Platforms):
  OPEL_CORSA_F = PlatformConfig(
    "OPEL CORSA F",
    [PSACarDocs("Vauxhall Corsa 2020")],
    CarSpecs(mass=1200, wheelbase=2.538, steerRatio=15.0),  # TODO: check steer ratio
    dbc_dict=dbc_dict('opel_corsa_f_hs1_generated', None, body_dbc='AEE2010_R3_HS2'),
  )


FW_QUERY_CONFIG = FwQueryConfig(
  # TODO: find firmware requests
  requests=[],
)

DBC = CAR.create_dbc_map()
