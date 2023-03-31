from cereal import car
from common.numpy_fast import clip
from opendbc.can.packer import CANPacker
from selfdrive.car import apply_std_steer_angle_limits
from selfdrive.car.ford import fordcan
from selfdrive.car.ford.values import CarControllerParams

VisualAlert = car.CarControl.HUDControl.VisualAlert


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)

    self.apply_curvature_last = 0
    self.steer_rate_limited = False
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False

  def update(self, CC, CS, frame):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)

    if CC.cruiseControl.cancel:
      # cancel stock ACC
      can_sends.append(fordcan.spam_cancel_button(self.packer))


    ### longitudinal control ###
    # send acc command at 50Hz
    if self.CP.openpilotLongitudinalControl and (frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

      precharge_brake = accel < -0.1
      if accel > -0.5:
        gas = accel
        decel = False
      else:
        gas = -5.0
        decel = True

      can_sends.append(fordcan.create_acc_command(self.packer, CC.longActive, gas, accel, precharge_brake, decel))


    ### lateral control ###
    # send steering commands at 20Hz
    if (frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        # apply limits to curvature and clip to signal range
        apply_curvature = apply_std_steer_angle_limits(actuators.curvature, self.apply_curvature_last, CS.out.vEgo, CarControllerParams)
        apply_curvature = clip(apply_curvature, -CarControllerParams.CURVATURE_MAX, CarControllerParams.CURVATURE_MAX)
      else:
        apply_curvature = 0.

      self.apply_curvature_last = apply_curvature
      can_sends.append(fordcan.create_lkas_command(self.packer))
      can_sends.append(fordcan.create_lat_ctl_msg(self.packer, CC.latActive, 0., 0., -apply_curvature, 0.))


    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)

    # send lkas ui command at 1Hz or if ui state changes
    if (frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_command(self.packer, main_on, CC.latActive, steer_alert, CS.lkas_status_stock_values))

    # send acc ui command at 20Hz or if ui state changes
    if (frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_acc_ui_command(self.packer, main_on, CC.latActive, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert

    new_actuators = actuators.copy()
    new_actuators.curvature = self.apply_curvature_last

    return new_actuators, can_sends
