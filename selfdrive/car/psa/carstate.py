from cereal import car
from opendbc.can.parser import CANParser
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import mean
from openpilot.selfdrive.car.psa.values import DBC, CarControllerParams
from openpilot.selfdrive.car.interfaces import CarStateBase

GearShifter = car.CarState.GearShifter
TransmissionType = car.CarParams.TransmissionType


class CarState(CarStateBase):
  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()

    # car speed
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FL'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_FR'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RL'],
      cp.vl['WHEEL_SPEEDS']['WHEEL_SPEED_RR'],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]) * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.yawRate = cp.vl['DYNAMICS']['YAW_RATE'] * CV.DEG_TO_RAD
    ret.standstill = False  # TODO

    # gas
    ret.gas = cp.vl['DRIVER']['GAS_PEDAL'] / 99.5
    ret.gasPressed = ret.gas > 1e-6  # TODO

    # brake
    ret.brake = cp.vl['DYNAMICS']['BRAKE_PRESSURE'] / 1500.
    ret.brakePressed = bool(cp.vl['BODY']['BRAKE_PRESSED'])
    ret.parkingBrake = bool(cp.vl['BODY']['PARKING_BRAKE'])  # TODO: check e-brake

    # steering wheel
    ret.steeringAngleDeg = cp.vl['STEERING_ALT']['ANGLE']
    ret.steeringRateDeg = cp.vl['STEERING_ALT']['RATE'] * cp.vl['STEERING_ALT']['RATE_SIGN']  # TODO: check units
    ret.steeringTorque = cp.vl['STEERING']['DRIVER_TORQUE']  # TODO: check units
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)  # TODO: adjust threshold
    ret.steerFaultTemporary = 0  # TODO
    ret.steerFaultPermanent = 0  # TODO
    ret.espDisabled = 0  # TODO

    # TODO: cruise state
    ret.cruiseState.speed = 0
    ret.cruiseState.enabled = False
    ret.cruiseState.available = False
    ret.cruiseState.nonAdaptive = True
    ret.cruiseState.standstill = False
    ret.accFaulted = False

    # gear
    # TODO: automatic transmission gear
    if self.CP.transmissionType == TransmissionType.manual:
      ret.clutchPressed = bool(cp.vl['BODY_2']['CLUTCH_PRESSED'])
      if bool(cp.vl['BODY']['REVERSE']):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # TODO: safety
    ret.stockFcw = 0
    ret.stockAeb = 0

    # button presses
    ret.leftBlinker = cp.vl['STEERING_COLUMN']['LEFT_INDICATOR']
    ret.rightBlinker = cp.vl['STEERING_COLUMN']['RIGHT_INDICATOR']

    # lock info
    ret.doorOpen = any([cp.vl['BODY']['DRIVER_DOOR'], cp.vl['BODY']['PASSENGER_DOOR']])
    ret.seatbeltUnlatched = cp.vl['RESTRAINTS']['DRIVER_SEATBELT'] != 2  # TODO: check LHD

    # stock signals from camera
    self.lka_stock_values = cp_cam.vl['LANE_KEEP_ASSIST']

    return ret

  @staticmethod
  def get_can_parser(CP):
    messages = [
      ('WHEEL_SPEEDS', 50),
      ('DYNAMICS', 100),
      ('DRIVER', 10),
      ('BODY', 20),
      ('BODY_2', 50),
      ('STEERING_COLUMN', 10),
      ('RESTRAINTS', 10),
    ]
    return CANParser(DBC[CP.carFingerprint]['pt'], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ('LANE_KEEP_ASSIST', 20),
    ]
    return CANParser(DBC[CP.carFingerprint]['pt'], messages, 2)
