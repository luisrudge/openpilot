#!/usr/bin/env python3
from collections import namedtuple
import csv
import math
import os

from pathlib import Path
from typing import List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from sklearn import linear_model  # pylint: disable=import-error

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
    ("VehRol_W_Actl", "Yaw_Data_FD1"),                    # vehicle roll angle (rad)
  ]
  checks = [
    # sig_address, frequency
    ("Yaw_Data_FD1", 1),
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
    ("LatCtlCurv_NoRate_Actl", "LateralMotionControl"),   # curvature rate (1/m^2)
    ("LatCtlCurv_No_Actl", "LateralMotionControl"),       # curvature (1/m)
  ]

  checks = [
    # sig_address, frequency
    ("LateralMotionControl", 20),
  ]

  return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, CANBUS.main)


FEATURES = [
  'carState.vEgo',
  'carState.steeringAngleDeg',
  'carState.steeringTorque',
  'carState.steeringPressed',

  # 'sendcan.LateralMotionControl.LatCtl_D_Rq',
  # 'sendcan.LateralMotionControl.LatCtlRampType_D_Rq',
  # 'sendcan.LateralMotionControl.LatCtlPrecision_D_Rq',
  # 'sendcan.LateralMotionControl.LatCtlPathOffst_L_Actl',
  'sendcan.LateralMotionControl.LatCtlPath_An_Actl',
  # 'sendcan.LateralMotionControl.LatCtlCurv_NoRate_Actl',
  # 'sendcan.LateralMotionControl.LatCtlCurv_No_Actl',

  'vehicleModel.roll',
  'vehicleModel.actualCurvature',

  'controlsState.desiredCurvature',
  'controlsState.lateralControlState.angleState.active',
  # 'controlsState.lateralControlState.angleState.steeringAngleDeg',
  # 'controlsState.lateralControlState.angleState.steeringAngleDesiredDeg',

  'actuatorsOutput.steeringAngleDeg',
]

X_labels = [
  # 'carState.vEgo',
  'sendcan.LateralMotionControl.LatCtlPath_An_Actl',
  'vehicleModel.roll',
]
Y_label = 'carState.steeringAngleDeg'

SECONDS = 100
MIN_TIME_ENGAGED = 3 * SECONDS
MIN_SEGMENT_TIME = 2 * SECONDS

Record = namedtuple('Record', [
  'carState',
  'sendcan',
  'vehicleModel',
  'controlsState',
  'actuatorsOutput',
])

def create_record() -> Record:
  return Record(
    carState=None,
    sendcan=None,
    vehicleModel=None,
    controlsState=None,
    actuatorsOutput=None,
  )


def collect(lr: MultiLogIterator) -> Tuple[int, List]:
  CP = CarInterface.get_params("FORD FOCUS 4TH GEN")
  VM = VehicleModel(CP)

  cp = get_can_parser(CP)
  cp_sent = get_sent_can_parser(CP)

  valid_cnt = 0

  record = create_record()
  row_list: List = []

  # state 'vEgo', 'steeringAngleDeg', 'steeringTorque', 'steeringPressed', 'actualCurvature'
  # actuators 'enabled', 'rampType', 'precision', 'curvature', 'pathOffset', 'pathAngle'

  def explode_dicts(**kwargs):
    for k, v in kwargs.items():
      if isinstance(v, dict):
        for k2, v2 in explode_dicts(**v):
          yield k + '.' + k2, v2
      else:
        yield k, v

  def try_append():
    nonlocal record
    if record.carState is not None and \
        record.sendcan is not None and \
        record.vehicleModel is not None and \
        record.controlsState is not None and \
        record.actuatorsOutput is not None:

      valid = record.controlsState['lateralControlState']['angleState']['active'] and not record.carState['steeringPressed']
      if valid:
        nonlocal valid_cnt
        valid_cnt += 1
        print(record)

      row = dict(explode_dicts(
        carState=record.carState,
        sendcan=record.sendcan,
        vehicleModel=record.vehicleModel,
        controlsState=record.controlsState,
        actuatorsOutput=record.actuatorsOutput,
      ))
      row_list.append(row)
      record = create_record()

  for msg in lr:

    if msg.which() == 'carState':
      carState = msg.carState

      # update recorded car state
      record = record._replace(carState = {
        'vEgo': carState.vEgo,
        'steeringAngleDeg': carState.steeringAngleDeg,
        'steeringTorque': carState.steeringTorque,
        'steeringPressed': carState.steeringPressed,
      })

    elif msg.which() == 'sendcan':
      # parse sent can messages
      can_string = msg.as_builder().to_bytes()
      cp_sent.update_string(can_string)

      # update recorded sendcan
      record = record._replace(sendcan = {k: v for k, v in cp_sent.vl.items() if v and isinstance(k, str)})

    elif msg.which() == 'liveParameters':
      # update vehicle model, used for curvature calculations
      liveParameters = msg.liveParameters

      # copied from controlsd
      x = max(liveParameters.stiffnessFactor, 0.1)
      sr = max(liveParameters.steerRatio, 0.1)
      VM.update_params(x, sr)

      vEgo = record.carState['vEgo'] if record.carState is not None else 0
      steeringAngleDeg = record.carState['steeringAngleDeg'] if record.carState is not None else 0

      # update recorded vehicle model
      record = record._replace(vehicleModel = {
        'roll': liveParameters.roll,
        'actualCurvature': -VM.calc_curvature(math.radians(steeringAngleDeg - liveParameters.angleOffsetDeg), vEgo, liveParameters.roll),
      })

    elif msg.which() == 'controlsState':
      # get desired curvature
      controlsState = msg.controlsState

      desiredCurvature = controlsState.desiredCurvature
      angleState = controlsState.lateralControlState.angleState

      # update recorded controls state
      record = record._replace(controlsState = {
        'desiredCurvature': desiredCurvature,
        'lateralControlState': {
          'angleState': {
            'active': angleState.active,
            'steeringAngleDeg': angleState.steeringAngleDeg,
            'steeringAngleDesiredDeg': angleState.steeringAngleDesiredDeg,
          },
        },
      })

    elif msg.which() == 'carControl':
      carControl = msg.carControl
      actuatorsOutput = carControl.actuatorsOutput

      # update recorded actuators output
      record = record._replace(actuatorsOutput = {
        'steeringAngleDeg': actuatorsOutput.steeringAngleDeg,
      })

    try_append()

  return valid_cnt, row_list


def plot(dataset: pd.DataFrame, title=None):
  fig, axs = plt.subplots(3, 2, figsize=(10, 10))
  if title is not None: fig.suptitle(title)

  # if 'carState.vEgo' in dataset:
  #   axs[0, 0].set_title('carState.vEgo')
  #   axs[0, 0].plot(dataset['carState.vEgo'])

  # if 'carState.steeringTorque' in dataset:
  #   axs[0, 0].set_title('carState.steeringTorque')
  #   axs[0, 0].plot(dataset['carState.steeringTorque'])

  if 'controlsState.desiredCurvature' in dataset and 'vehicleModel.actualCurvature' in dataset:
    axs[0, 0].set_title('controlsState.desiredCurvature')
    axs[0, 0].plot(dataset['controlsState.desiredCurvature'], label='desiredCurvature')
    axs[0, 0].plot(dataset['vehicleModel.actualCurvature'], label='actualCurvature')
    axs[0, 0].legend()

  if 'carState.steeringPressed' in dataset and 'controlsState.lateralControlState.angleState.active' in dataset:
    axs[1, 0].set_title('active')
    axs[1, 0].plot(dataset['controlsState.lateralControlState.angleState.active'], label='enabled')
    axs[1, 0].plot(dataset['carState.steeringPressed'], label='steeringPressed')
    axs[1, 0].legend()

  if 'actuatorsOutput.steeringAngleDeg' in dataset and 'carState.steeringAngleDeg' in dataset:
    axs[0, 1].set_title('steeringAngleDeg')
    axs[0, 1].plot(dataset['actuatorsOutput.steeringAngleDeg'], label='desired (actuatorsOutput)')
    axs[0, 1].plot(dataset['carState.steeringAngleDeg'], label='actual (carState)')
    axs[0, 1].legend()

    # # get gradient of both steeringAngleDeg
    # grad_actuatorsOutput = np.gradient(dataset['actuatorsOutput.steeringAngleDeg'])
    # grad_carState = np.gradient(dataset['carState.steeringAngleDeg'])

    # axs[1, 1].set_title('gradient steeringAngleDeg')
    # axs[1, 1].plot(grad_actuatorsOutput, label='d actuatorsOutput.steeringAngleDeg')
    # axs[1, 1].plot(grad_carState, label='d carState.steeringAngleDeg')
    # axs[1, 1].legend()

  if 'sendcan.LateralMotionControl.LatCtlPath_An_Actl' in dataset:
    axs[1, 1].set_title('LateralMotionControl.LatCtlPath_An_Actl')
    axs[1, 1].plot(dataset['sendcan.LateralMotionControl.LatCtlPath_An_Actl'], label='LatCtlPath_An_Actl')
    # if 'carState.steeringAngleDeg' in dataset:
    #   dataset['carState.steeringAngleRad'] = np.radians(dataset['carState.steeringAngleDeg'])
    #   dataset['carState.pathAngleRad'] = dataset['carState.steeringAngleRad'] / 13.8
    #   axs[1, 1].plot(np.radians(dataset['carState.pathAngleRad']), label='carState.pathAngleRad')
    axs[1, 1].legend()

  if 'vehicleModel.roll' in dataset:
    axs[2, 1].set_title('vehicleModel.roll')
    axs[2, 1].plot(dataset['vehicleModel.roll'], label='vehicleModel.roll')
    axs[2, 1].legend()


def prepare(dataset: pd.DataFrame) -> Tuple[pd.DataFrame, List[pd.DataFrame]]:
  segments: List[pd.DataFrame] = []
  rows = []
  all_rows = []

  def extra(df: pd.DataFrame) -> pd.DataFrame:
    # df['steeringAngleDeltaDeg'] = df['carState.steeringAngleDeg'] - df['actuatorsOutput.steeringAngleDeg']
    return df

  def build_df():
    nonlocal segments, rows

    if len(rows) >= MIN_SEGMENT_TIME:
      # create new dataframe from processed rows
      df = pd.DataFrame(rows, columns=FEATURES)
      segments.append(extra(df))
      all_rows.extend(rows)

    rows = []

  # iterate rows of dataset
  for i, row in dataset.iterrows():
    # is active and not steering pressed
    active = row['controlsState.lateralControlState.angleState.active'] and not row['carState.steeringPressed']
    if not active:
      active_frames = 0
      build_df()
      continue

    active_frames += 1

    # only add new row if active for at least min time
    if active_frames < MIN_TIME_ENGAGED:
      continue

    rows.append(row[FEATURES])

  build_df()

  # build "superset" of all valid rows
  df = pd.DataFrame(all_rows, columns=FEATURES)
  return extra(df), segments


if __name__ == "__main__":
  # use caching
  os.environ["FILEREADER_CACHE"] = "1"

  ROUTES = [
    # "86d00e12925f4df7|2022-06-18--11-45-12",  # 18th June - Lancaster to Liverpool

    # "86d00e12925f4df7|2022-06-23--16-41-03",  # 23rd June - Lancaster to Blackpool

    # 23rd June - Car park joystick testing
    # "86d00e12925f4df7|2022-06-23--19-16-14",
    # "86d00e12925f4df7|2022-06-23--19-18-21",
    # "86d00e12925f4df7|2022-06-23--19-21-24",
    # "86d00e12925f4df7|2022-06-23--19-24-26",
    # "86d00e12925f4df7|2022-06-23--19-38-33",
    # "86d00e12925f4df7|2022-06-23--19-40-39",
    # "86d00e12925f4df7|2022-06-23--19-43-11",
    # "86d00e12925f4df7|2022-06-23--19-45-55",
    # "86d00e12925f4df7|2022-06-23--19-52-33",
    # "86d00e12925f4df7|2022-06-23--19-55-05",
    # "86d00e12925f4df7|2022-06-23--20-03-47",
    # "86d00e12925f4df7|2022-06-23--20-24-01",
    # "86d00e12925f4df7|2022-06-23--20-34-32",
    # "86d00e12925f4df7|2022-06-23--20-46-04",
    # "86d00e12925f4df7|2022-06-23--20-49-13",
    # "86d00e12925f4df7|2022-06-23--20-56-06",
    # "86d00e12925f4df7|2022-06-23--20-59-11",

    # "86d00e12925f4df7|2022-07-16--22-20-26",  # 16th July - ford-angle-experiment

    # "86d00e12925f4df7|2022-07-19--15-02-16",  # 19th July - ford-angle-experiment with revisions

    # "86d00e12925f4df7|2022-07-20--18-12-24",  # 20th July - manchester to home

    "86d00e12925f4df7|2022-07-21--13-05-25",  # 21st July - home to lancaster
  ]

  CACHE_LOGS = True

  all_combined = []

  for ROUTE in ROUTES:
    print(f"Processing route {ROUTE}")
    r = Route(ROUTE)
    csv_path = f"{r.name.time_str}.csv"


    # Pull data from route
    if not Path(csv_path).is_file() or not CACHE_LOGS:
      # Collect data from route
      lr = MultiLogIterator(r.log_paths())

      try:
        valid_cnt, row_list = collect(lr)
        # print('valid_cnt', valid_cnt, 'len(row_list)', len(row_list))
      except Exception as e:
        print(f"Error collecting data from route {ROUTE}: {e}")
        continue


      # Write data to CSV
      with open(csv_path, 'w', encoding='utf8', newline='') as f:
        fc = csv.DictWriter(f, fieldnames=row_list[0].keys())
        fc.writeheader()
        fc.writerows(row_list)


    # Prepare data for analysis
    if True:
      r = Route(ROUTE)
      print(f"Loading data from route {r.name}")

      # read csv
      dataset = pd.read_csv(csv_path)
      # print(f"Got {len(dataset)} rows")
      # print(dataset.head())
      # plot(dataset, title=f"Route {r.name}, Length {(len(dataset)/100.):.2f}s")

      # prepare dataset by splitting into valid segments for analysis
      combined, segments = prepare(dataset)

      plot(combined, title=f"Combined all active segments, Length {(len(combined)/100.):.2f}s")

      with open(f"{r.name.time_str}--combined.csv", 'w', encoding='utf8', newline='') as f:
        fc = csv.DictWriter(f, fieldnames=combined.columns)
        fc.writeheader()
        fc.writerows(combined.to_dict('records'))

      for i, segment in enumerate(segments):
        # if len(segment) > 10 * SECONDS:
        #   plot(segment, title=f"Segment {i+1}/{len(segments)}, Length {(len(segment)/100.):.2f}s")

        with open(f"{r.name.time_str}--{i}.csv", 'w', encoding='utf8', newline='') as f:
          fc = csv.DictWriter(f, fieldnames=segment.columns)
          fc.writeheader()
          fc.writerows(segment.to_dict('records'))

      plt.show()


    if True:
      r = Route(ROUTE)
      print(f"Loading data from route {r.name}")

      segments = []
      for f in os.listdir(os.getcwd()):
        if not f.startswith(f"{r.name.time_str}--"):
          continue

        try:
          segment_num = int(f.split('--')[-1].split('.')[0])
        except:
          continue

        # read csv
        dataset = pd.read_csv(f)
        segments.append(dataset)

      print(f"got {len(segments)} segments")
      for i, segment in enumerate(segments):
        print(f"Segment {i+1}/{len(segments)}")
        print(segment.head())
        # if len(segment) > 8 * SECONDS:
        #   plot(segment, title=f"Segment {i+1}/{len(segments)}, Length {(len(segment)/100.):.2f}s")
      plt.show()

      # fit model
      if len(segments) > 0:
        combined = pd.concat(segments)
        # print(f"Fitting combined segments")
        # model = linear_model.LinearRegression()
        # model.fit(combined[X_labels], combined[Y_label])
        # print(f"Coefficients: {model.coef_}")
        # print(f"Intercept: {model.intercept_}")
        # print(f"R2: {model.score(combined[X_labels], combined[Y_label])}")
        all_combined.append(combined)

      # carState.steeringAngleDeg = 174.859 * pathAngleRad + 21.4685 * roll
      # 174.859 * pathAngleRad = carState.steeringAngleDeg - 21.4685 * roll
      # pathAngleRad = (carState.steeringAngleDeg - 21.4685 * roll) / 174.859

      # pathAngleRad = carState.steeringAngleDeg / 163.52707978 + roll * 25.4976824

      # pathAngleRad ~~= carState.steeringAngleDeg / 144.46264391
      # pathAngleRad ~~= carState.steeringAngleDeg * (pi / 180) / 2.521348782
      # pathAngleRad ~~= carState.steeringAngleDeg * (pi / 180) / (0.182706434 * steerRatio)

      # carState.steeringAngleDeg = 147.15600605 * pathAngleRad + 23.22011968 * roll
      # carState.steeringAngleDeg ~= 147.15600605 * pathAngleRad
      # pathAngleRad = carState.steeringAngleDeg / 147.15600605
      # pathAngleRad = carState.steeringAngleDeg * (pi / 180) / 2.56835682
      # pathAngleRad = carState.steeringAngleDeg * (pi / 180) / 5.373085193 * steerRatio

      # pathAngleRad ~= carState.steeringAngleDeg / 174.859
      # pathAngleRad ~= carState.steeringAngleDeg * (pi/180) / 3.05186799
      # pathAngleRad ~= carState.steeringAngleDeg * (pi/180) / (0.221149854 * steerRatio)

  print()
  print(f"Fitting all combined routes")
  all_combined = pd.concat(all_combined)
  model = linear_model.LinearRegression()
  model.fit(all_combined[X_labels], all_combined[Y_label])
  print(f"Coefficients: {model.coef_}")
  print(f"Intercept: {model.intercept_}")
  print(f"R2: {model.score(all_combined[X_labels], all_combined[Y_label])}")

  # plot
  plt.plot(all_combined[X_labels], all_combined[Y_label], 'o', label='data')
  plt.plot(all_combined[X_labels], model.predict(all_combined[X_labels]), label='model')
  plt.xlabel(X_labels[0])
  plt.ylabel(Y_label)
  plt.legend()
  plt.show()
