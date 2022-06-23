#!/usr/bin/env python3
import csv
import math
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from selfdrive.car.ford.interface import CarInterface
from selfdrive.car.ford.values import CANBUS, DBC
from selfdrive.controls.lib.vehicle_model import VehicleModel

from tools.lib.route import Route, SegmentName
from tools.lib.logreader import MultiLogIterator


def get_can_parser(CP):
  signals = [
    # sig_name, sig_address
    ("Veh_V_ActlEng", "EngVehicleSpThrottle2"),            # ABS vehicle speed (kph)
    # ("VehYaw_W_Actl", "Yaw_Data_FD1"),                     # ABS vehicle yaw rate (rad/s)
    # ("ApedPos_Pc_ActlArb", "EngVehicleSpThrottle"),        # PCM throttle (pct)
    ("StePinComp_An_Est", "SteeringPinion_Data"),          # PSCM estimated steering angle (deg)
                                                            # Calculates steering angle (and offset) from pinion
                                                            # angle and driving measurements.
                                                            # StePinRelInit_An_Sns is the pinion angle, initialised
                                                            # to zero at the beginning of the drive.
    # ("SteeringColumnTorque", "EPAS_INFO"),                 # PSCM steering column torque (Nm)
    ("LaHandsOff_B_Actl", "Lane_Assist_Data3_FD1"),        # PSCM LKAS hands off wheel
  ]

  checks = [
    # sig_address, frequency
    ("EngVehicleSpThrottle2", 50),
    ("Yaw_Data_FD1", 100),
    ("EngVehicleSpThrottle", 100),
    ("SteeringPinion_Data", 100),
    ("EPAS_INFO", 50),
    ("Lane_Assist_Data3_FD1", 33),
  ]

  return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CANBUS.main)


def get_sent_can_parser(CP):
  signals = [
    # sig_name, sig_address
    ("LatCtl_D_Rq", "LateralMotionControl"),              # LCA/TJA request
    ("LatCtlRampType_D_Rq", "LateralMotionControl"),      # ramp speed
    ("LatCtlPrecision_D_Rq", "LateralMotionControl"),     # precision
    ("LatCtlPathOffst_L_Actl", "LateralMotionControl"),   # path offset (m)
    ("LatCtlPath_An_Actl", "LateralMotionControl"),       # path angle (rad)
    # ("LatCtlCurv_NoRate_Actl", "LateralMotionControl"),   # curvature rate (1/m^2)
    ("LatCtlCurv_No_Actl", "LateralMotionControl"),       # curvature (1/m)
  ]

  checks = [
    # sig_address, frequency
    ("LateralMotionControl", 20),
  ]

  return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CANBUS.main)

FEATURES = [
  'state.steeringAngleDeg',
  'state.vEgoRaw',
  'actuators.rampType',
  'actuators.precision',
  'actuators.curvature',
  'actuators.pathOffset',
  'actuators.pathAngle',
  'vehicleModel.actualCurvature',
  'controlsState.desiredCurvature',
  'controlsState.desiredSteeringAngleDeg',
]

X_labels = [
  # 'state.vEgoRaw',
  # 'actuators.rampType',
  # 'actuators.precision',
  # 'actuators.curvature',
  'actuators.pathAngle',
  # 'actuators.pathOffset',
]
Y_label = 'state.steeringAngleDeg'

MIN_TIME_ENGAGED = 3 * 100
MIN_SEGMENT_TIME = 1 * 100


def collect(lr):
  CP = CarInterface.get_params("FORD FOCUS 4TH GEN")
  VM = VehicleModel(CP)

  cp = get_can_parser(CP)
  cp_sent = get_sent_can_parser(CP)

  params = None
  state = None
  actuators = None
  vehicleModel = None
  controlsState = None

  engaged = False
  steering_pressed = False
  valid_cnt = 0

  row_list = []

  # state 'vEgoRaw', 'yawRate', 'gas', 'steeringAngleDeg', 'steeringTorque', 'steeringPressed'
  # actuators 'enabled', 'rampType', 'precision', 'curvature', 'curvatureRate', 'pathOffset', 'pathAngle'

  def explode_dicts(**kwargs):
    for k, v in kwargs.items():
      if isinstance(v, dict):
        for k2, v2 in explode_dicts(**v):
          yield k + '.' + k2, v2
      else:
        yield k, v

  def try_append():
    nonlocal state, actuators, vehicleModel, controlsState
    if state is not None and actuators is not None and vehicleModel is not None and controlsState is not None:
      valid = engaged and not steering_pressed
      if valid:
        nonlocal valid_cnt
        valid_cnt += 1
        print('state', state)
        print('actuators', actuators)
        print('vehicleModel', vehicleModel)
        print('controlsState', controlsState)

      row = dict(explode_dicts(state=state, actuators=actuators, vehicleModel=vehicleModel, controlsState=controlsState))
      row_list.append(row)
      state = None
      actuators = None
      vehicleModel = None
      controlsState = None

  for msg in lr:

    if msg.which() == 'liveParameters':
      # update vehicle model, used for curvature calculations
      params = msg.liveParameters

      # copied from controlsd
      x = max(params.stiffnessFactor, 0.1)
      sr = max(params.steerRatio, 0.1)
      VM.update_params(x, sr)

      vEgoRaw = state['vEgoRaw'] if state is not None else 0
      steeringAngleDeg = state['steeringAngleDeg'] if state is not None else 0

      vehicleModel = {
        'actualCurvature': -VM.calc_curvature(math.radians(steeringAngleDeg - params.angleOffsetDeg), vEgoRaw, params.roll),
      }

    elif msg.which() == 'controlsState':
      # get desired curvature
      desiredCurvature = msg.controlsState.desiredCurvature
      vEgoRaw = state['vEgoRaw'] if state is not None else 0
      roll = params.roll if params is not None else 0

      controlsState = {
        'desiredCurvature': desiredCurvature,
        'desiredSteeringAngleDeg': math.degrees(VM.get_steer_from_curvature(-desiredCurvature, vEgoRaw, roll)),
      }

    elif msg.which() == 'can':
      # parse can messages
      can_string = msg.as_builder().to_bytes()
      cp.update_string(can_string)

      # build state
      def update(cp):
        ret = {}

        # car speed
        ret['vEgoRaw'] = cp.vl["EngVehicleSpThrottle2"]["Veh_V_ActlEng"] * CV.KPH_TO_MS
        # ret['yawRate'] = cp.vl["Yaw_Data_FD1"]["VehYaw_W_Actl"] * CV.RAD_TO_DEG

        # gas pedal
        # ret['gas'] = cp.vl["EngVehicleSpThrottle"]["ApedPos_Pc_ActlArb"] / 100.

        # steering wheel
        ret['steeringAngleDeg'] = cp.vl["SteeringPinion_Data"]["StePinComp_An_Est"]
        # ret['steeringTorque'] = cp.vl["EPAS_INFO"]["SteeringColumnTorque"]
        ret['steeringPressed'] = cp.vl["Lane_Assist_Data3_FD1"]["LaHandsOff_B_Actl"] == 0

        return ret

      state = update(cp)
      steering_pressed = state['steeringPressed']

    elif msg.which() == 'sendcan':
      # parse sent can messages
      can_string = msg.as_builder().to_bytes()
      cp_sent.update_string(can_string)

      # build state
      def update(cp_cam):
        ret = {}

        ret['enabled'] = cp_cam.vl["LateralMotionControl"]["LatCtl_D_Rq"] == 1

        ret['rampType'] = cp_cam.vl["LateralMotionControl"]["LatCtlRampType_D_Rq"]
        ret['precision'] = cp_cam.vl["LateralMotionControl"]["LatCtlPrecision_D_Rq"]

        ret['curvature'] = cp_cam.vl["LateralMotionControl"]["LatCtlCurv_No_Actl"]
        # ret['curvatureRate'] = cp_cam.vl["LateralMotionControl"]["LatCtlCurv_NoRate_Actl"]
        ret['pathOffset'] = cp_cam.vl["LateralMotionControl"]["LatCtlPathOffst_L_Actl"]
        ret['pathAngle'] = cp_cam.vl["LateralMotionControl"]["LatCtlPath_An_Actl"]

        return ret

      actuators = update(cp_sent)
      engaged = actuators['enabled']

    # elif msg.which() == 'liveLocationKalman':
    #   # get actual curvature

    #   llk = msg.liveLocationKalman

    #   actual_curvature_vm = -VM.calc_curvature(math.radians(state['steeringAngleDeg'] - params.angleOffsetDeg), state['vEgoRaw'], params.roll)
    #   actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / state['vEgoRaw']

    #   vehicleModel = {
    #     'actualCurvature': interp(state['vEgoRaw'], [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
    #   }

    try_append()

  return valid_cnt, row_list


def plot(dataset, title=None):
  # plot state in left column and actuators in right column
  # each with seperate subplots/rows
  # vEgoRaw            curvature
  # steeringAngleDeg   pathOffset
  # steeringPressed    pathAngle

  fig, axs = plt.subplots(4, 2, figsize=(10, 10))
  if title is not None: fig.suptitle(title)

  if 'state.vEgoRaw' in dataset:
    axs[0, 0].set_title("state.vEgoRaw")
    axs[0, 0].plot(dataset['state.vEgoRaw'])

  if 'state.steeringPressed' in dataset and 'actuators.enabled' in dataset:
    axs[1, 0].set_title("active")
    axs[1, 0].plot(dataset['actuators.enabled'], label='enabled')
    axs[1, 0].plot(dataset['state.steeringPressed'], label='steeringPressed')
    axs[1, 0].legend()

  # if 'actuators.curvature' in dataset:
  #   axs[0, 1].set_title("curvature")
  #   axs[0, 1].plot(dataset['actuators.curvature'])

  if 'actuators.pathAngle' in dataset:
    axs[2, 0].set_title("actuators.pathAngle")
    axs[2, 0].plot(dataset['actuators.pathAngle'], label='pathAngle')
    # axs[2, 0].plot(dataset['state.steeringAngleDeg'], label='steeringAngleDeg')
    # axs[2, 0].legend()

  # if 'actuators.pathOffset' in dataset:
  #   axs[1, 1].set_title("actuators.pathOffset")
  #   axs[1, 1].plot(dataset['actuators.pathOffset'])

  if 'controlsState.desiredCurvature' in dataset:
    axs[0, 1].set_title("controlsState.desiredCurvature")
    axs[0, 1].plot(dataset['controlsState.desiredCurvature'])

  if 'vehicleModel.actualCurvature' in dataset:
    axs[1, 1].set_title("vehicleModel.actualCurvature")
    axs[1, 1].plot(dataset['vehicleModel.actualCurvature'])

  if 'controlsState.desiredSteeringAngleDeg' in dataset:
    axs[2, 1].set_title("controlsState.desiredSteeringAngleDeg")
    axs[2, 1].plot(dataset['controlsState.desiredSteeringAngleDeg'])

  if 'state.steeringAngleDeg' in dataset:
    axs[3, 1].set_title("state.steeringAngleDeg")
    axs[3, 1].plot(dataset['state.steeringAngleDeg'])


def prepare(dataset):
  segments = []
  rows = []
  all_rows = []

  def build_df():
    nonlocal segments, rows

    if len(rows) >= MIN_SEGMENT_TIME:
      # create new dataframe from processed rows
      df = pd.DataFrame(rows, columns=FEATURES)
      segments.append(df)
      all_rows.extend(rows)

    rows = []

  # iterate rows of dataset
  for i, row in dataset.iterrows():
    # is active and not steering pressed
    active = row['actuators.enabled'] and not row['state.steeringPressed']
    if not active:
      active_frames = 0
      build_df()
      continue

    active_frames += 1

    # only add new row if active for at least 5 seconds
    if active_frames < MIN_TIME_ENGAGED:
      continue

    rows.append(row[FEATURES])

  build_df()

  # build "superset" of all valid rows
  df = pd.DataFrame(all_rows, columns=FEATURES)
  segments.insert(0, df)
  return segments


def standardise(dataset):
  """
  Standardise the dataset features
  """
  from sklearn.preprocessing import StandardScaler

  scaler = StandardScaler()
  scaler.fit(dataset)

  return scaler.transform(dataset)


def fit(dataset):
  """
  Fit the dataset to a model
  """
  from sklearn.linear_model import LinearRegression
  from sklearn.model_selection import train_test_split

  df = dataset.copy()
  columns = list(df.columns)

  df['state.steeringAngleDeg'] = df['state.steeringAngleDeg'].apply(lambda x: x if abs(x) <= 43. else None)
  df = df.dropna()

  X = df[['actuators.pathAngle']] #, '1']]
  # X.drop(columns=['state.steeringAngleDeg'], inplace=True)

  y = df['state.steeringAngleDeg'].values
  print(max(y))

  print("X:")
  print(X.size, X)
  print("Y:")
  print(y.size, y)

  # split to train and test
  X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.5, random_state=42)

  # fit the model
  model = LinearRegression(fit_intercept=True)
  model.fit(X_train, y_train)

  # print results
  print("")
  print("Coefficients:")
  for i, coef in enumerate(model.coef_):
    print(f"  {columns[i]}: {coef} (inverse {1 / coef})")
  print(f"c * 360 / 2pi: {model.coef_[0] * 360. / (2. * math.pi)}")
  # print(f"c * (360 / 2pi) / (13.7284): {model.coef_[0] * (360. / (2. * math.pi)) / (13.7284)}")

  print(f"Intercept: {model.intercept_}")
  print("")

  # make predictions using testing set
  y_pred = model.predict(X_test)

  plt.figure()
  plt.scatter(X_test.loc[:, 'actuators.pathAngle'], y_test, color="black", s=1)
  plt.plot(X_test.loc[:, 'actuators.pathAngle'], y_pred, color="blue", linewidth=2)

  plt.title(f"y = {2 * math.pi * model.coef_[0] / 360.} * math.degrees(x) + math.degrees({model.intercept_ * (2 * math.pi) / 360.})")
  # y = mdeg(x) + deg(b)
  # y - deg(b) = mdeg(x)
  # (y - deg(b)) / m = deg(x)
  # radians((y - deg(b)) / m) = x

  plt.xlabel('actuators.pathAngle (radians)')
  plt.ylabel('state.steeringAngleDeg')

  # X_super = np.asarray([-0.5, 0.5]).reshape(-1, 1)
  # y_super = model.predict(X_super)
  # print(X_super, y_super)
  # plt.scatter(X_super, y_super, color="red", s=1)

  # print("mean square error:", np.mean((model.predict(X_test) - y_test) ** 2))
  # print("test score:", model.score(X_test, y_test))

  return model


if __name__ == "__main__":
  # use caching
  os.environ["FILEREADER_CACHE"] = "1"

  if "--collect" in sys.argv:
    """pull data from route"""

    r = Route(sys.argv[-1])
    lr = MultiLogIterator(r.log_paths())

    valid_cnt, row_list = collect(lr)
    print('valid_cnt', valid_cnt, 'len(row_list)', len(row_list))

    with open(f"{r.name.time_str}.csv", 'w', encoding='utf8', newline='') as f:
      headings = row_list[0].keys()
      print('headings:', headings)
      fc = csv.DictWriter(f, fieldnames=headings)
      fc.writeheader()
      fc.writerows(row_list)

  if "--prepare" in sys.argv:
    """prepare the data"""

    r = Route(sys.argv[-1])
    print(f"Loading data from route {r.name}")

    # read csv
    dataset = pd.read_csv(f"{r.name.time_str}.csv")
    print(f"Got {len(dataset)} rows")
    print(dataset.head())
    plot(dataset, title=f"Route {r.name}, Length {(len(dataset)/100.):.2f}s")

    # prepare dataset by splitting into valid segments for analysis
    segments = prepare(dataset)
    supersegment, segments = segments[0], segments[1:]

    plot(supersegment, title=f"Combined segments, Length {(len(supersegment)/100.):.2f}s")

    with open(f"{r.name.time_str}--all.csv", 'w', encoding='utf8', newline='') as f:
      fc = csv.DictWriter(f, fieldnames=supersegment.columns)
      fc.writeheader()
      fc.writerows(supersegment.to_dict('records'))

    for i, segment in enumerate(segments):
      # plot(segment, title=f"Segment {i+1}/{len(segments)}, Length {(len(segment)/100.):.2f}s")

      with open(f"{r.name.time_str}--{i}.csv", 'w', encoding='utf8', newline='') as f:
        fc = csv.DictWriter(f, fieldnames=segment.columns)
        fc.writeheader()
        fc.writerows(segment.to_dict('records'))

    plt.show()

  if "--fit" in sys.argv:
    """fit the model"""
    segment_name = sys.argv[-1]
    print(f"Loading data from route segment {segment_name}")

    columns = []
    columns.extend(X_labels)
    columns.append(Y_label)
    print('desired columns:', columns)

    # read csv
    dataset = pd.read_csv(f"{segment_name}.csv")
    print('actual columns: ', list(dataset.columns))
    missing_columns = [c for c in list(dataset.columns) if c not in columns]
    dataset.drop(missing_columns, axis=1, inplace=True)
    print(f"Got {len(dataset)} rows")
    print(dataset.head())
    plot(dataset, title=f"Segment {segment_name}")

    # new_columns = {
    #   'state.steeringAngleDeg': 'state.steeringAngleDeg',
    #   'actuators.pathAngle': 'actuators.pathAngle',
    #   # 'state.vEgoRaw actuators.pathAngle': ['state.vEgoRaw', 'actuators.pathAngle'],
    #   # 'state.vEgoRaw^2 actuators.pathAngle': ['state.vEgoRaw', 'state.vEgoRaw', 'actuators.pathAngle'],
    #   # 'state.vEgoRaw': 'state.vEgoRaw',
    #   # 'state.steeringAngleDeg': 'state.steeringAngleDeg',
    #   # 'state.vEgoRaw^2': ['state.vEgoRaw', 'state.vEgoRaw'],
    #   # 'actuators.pathAngle^2': ['actuators.pathAngle', 'actuators.pathAngle'],
    #   # 'state.vEgoRaw^2 actuators.pathAngle^2': ['state.vEgoRaw', 'state.vEgoRaw', 'actuators.pathAngle', 'actuators.pathAngle'],
    #   # 'vehicleModel.actualCurvature': 'vehicleModel.actualCurvature',
    #   # 'actuators.curvature': 'actuators.curvature',
    #   # 'actuators.pathOffset': 'actuators.pathOffset',
    #   # 'actuators.curvature^2': ['actuators.curvature', 'actuators.curvature'],
    #   # 'actuators.pathOffset^2': ['actuators.pathOffset', 'actuators.pathOffset'],
    #   # 'state.vEgoRaw actuators.curvature': ['state.vEgoRaw', 'actuators.curvature'],
    #   # 'state.vEgoRaw actuators.pathOffset': ['state.vEgoRaw', 'actuators.pathOffset'],
    #   # 'state.vEgoRaw^2 actuators.curvature': ['state.vEgoRaw', 'state.vEgoRaw', 'actuators.curvature'],
    #   # 'state.vEgoRaw^2 actuators.pathOffset': ['state.vEgoRaw', 'state.vEgoRaw', 'actuators.pathOffset'],
    #   # 'state.vEgoRaw^2 actuators.curvature^2': ['state.vEgoRaw', 'state.vEgoRaw', 'actuators.curvature', 'actuators.curvature'],
    #   # 'state.vEgoRaw^2 actuators.pathOffset^2': ['state.vEgoRaw', 'state.vEgoRaw', 'actuators.pathOffset', 'actuators.pathOffset'],
    #   '1': 1,
    # }

    # def action_to_transformer(label):
    #   if isinstance(label, int):
    #     return lambda x: label

    #   elif isinstance(label, str):
    #     return lambda x: x[columns.index(label)]

    #   elif isinstance(label, list):
    #     def v(x, y):
    #       a = label[y]
    #       if isinstance(a, str):
    #         return x[columns.index(a)]
    #       else:
    #         return y

    #     if len(label) == 2:
    #       return lambda x: v(x, 0) * v(x, 1)
    #     elif len(label) == 3:
    #       return lambda x: v(x, 0) * v(x, 1) * v(x, 2)
    #     elif len(label) == 4:
    #       return lambda x: v(x, 0) * v(x, 1) * v(x, 2) * v(x, 3)

    # column_transformers = {label: action_to_transformer(action) for label, action in new_columns.items()}

    # new_rows = []
    # for i, row in dataset.iterrows():
    #   new_row = [transformer(row) for transformer in column_transformers.values()]
    #   new_rows.append(new_row)

    df = dataset.copy()
    df['1'] = 1

    # df = pd.DataFrame(new_rows, columns=column_transformers.keys())
    print()
    print("Transformed data:")
    print(df.head())

    # fit model
    model = fit(df)

    plt.show()
