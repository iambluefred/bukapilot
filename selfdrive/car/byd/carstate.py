from cereal import car
from collections import deque
from math import ceil
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.byd.values import DBC, CAR, HUD_MULTIPLIER
from time import time

from common.features import Features

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["DRIVE_STATE"]['GEAR']
    self.set_distance_values = can_define.dv['ACC_HUD_ADAS']['SET_DISTANCE']
    self.is_cruise_latch = False
    self.prev_angle = 0

  def update(self, cp):
    ret = car.CarState.new_message()
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FR'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_BL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_BR'], # TODO: why would BR make the value wrong? Wheelspeed sensor prob?
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])

    # unfiltered speed from CAN sensors
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # safety checks to engage
    can_gear = int(cp.vl["DRIVE_STATE"]['GEAR'])

    ret.doorOpen = any([cp.vl["METER_CLUSTER"]['BACK_LEFT_DOOR'],
                     cp.vl["METER_CLUSTER"]['FRONT_LEFT_DOOR'],
                     cp.vl["METER_CLUSTER"]['BACK_RIGHT_DOOR'],
                     cp.vl["METER_CLUSTER"]['FRONT_RIGHT_DOOR']])

    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]['SEATBELT_DRIVER'] == 0
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.brakeHoldActive = False # TODO bool(cp.vl["PARKING_BRAKE"]["CAR_ON_HOLD"])

    disengage = ret.doorOpen or ret.seatbeltUnlatched or ret.brakeHoldActive
    if disengage:
      self.is_cruise_latch = False

    # gas pedal
    ret.gas = cp.vl["PEDAL"]['GAS_PEDAL']
    ret.gasPressed = ret.gas > 0.01

    # brake pedal
    ret.brake = cp.vl["PEDAL"]['BRAKE_PEDAL']
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"])

    # steer
    ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]['STEER_ANGLE_2']
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE']
    ret.steeringTorqueEps = cp.vl["STEER_MODULE_2"]['DRIVER_EPS_TORQUE'] * steer_dir
    ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 5) 
    ret.steerWarning = False # TODO
    ret.steerError = False   # TODO

    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    # TODO: get the real value
    ret.stockAeb = False
    ret.stockFcw = False
    ret.cruiseState.available = any([cp.vl["ACC_HUD_ADAS"]["ACC_ON1"], cp.vl["ACC_HUD_ADAS"]["ACC_ON2"]]) # TODO need two?

    distance_val = int(cp.vl["ACC_HUD_ADAS"]['SET_DISTANCE'])
    ret.cruiseState.setDistance = self.parse_set_distance(self.set_distance_values.get(distance_val, None))

    # engage and disengage logic
    if (cp.vl["PCM_BUTTONS"]["SET_BTN"] != 0 or cp.vl["PCM_BUTTONS"]["RES_BTN"] != 0) and not ret.brakePressed:
      self.is_cruise_latch = True

    # set speed in range of 30 - 130kmh only
    ret.cruiseState.speedCluster = max(int(cp.vl["ACC_HUD_ADAS"]['SET_SPEED']), 30) * CV.KPH_TO_MS
    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = bool(cp.vl["ACC_CMD"]["STANDSTILL_STATE"])
    ret.cruiseState.nonAdaptive = False

    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    if ret.brakePressed:
      self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch

    # button presses
    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])
    ret.genericToggle = bool(cp.vl["STALKS"]["GENERIC_TOGGLE"])
    ret.espDisabled = False

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      ret.leftBlindspot = bool(cp.vl["BSM"]["LEFT_APPROACH"])
      ret.rightBlindspot = bool(cp.vl["BSM"]["RIGHT_APPROACH"])

    return ret


  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("GEAR", "DRIVE_STATE", 1),
      ("BRAKE_PRESSED", "DRIVE_STATE", 0),
      ("SET_DISTANCE", "ACC_HUD_ADAS", 1),
      ("SET_SPEED", "ACC_HUD_ADAS", 0),
      ("ACC_ON1", "ACC_HUD_ADAS", 0),
      ("ACC_ON2", "ACC_HUD_ADAS", 0),
      ("WHEELSPEED_FR", "WHEEL_SPEED", 0.),
      ("WHEELSPEED_FL", "WHEEL_SPEED", 0.),
      ("WHEELSPEED_BR", "WHEEL_SPEED", 0.),
      ("WHEELSPEED_BL", "WHEEL_SPEED", 0.),
      ("GAS_PEDAL", "PEDAL", 0.),
      ("BRAKE_PEDAL", "PEDAL", 0.),
      ("SEATBELT_DRIVER", "METER_CLUSTER", 0),
      ("BACK_RIGHT_DOOR", "METER_CLUSTER", 0),
      ("BACK_LEFT_DOOR", "METER_CLUSTER", 0),
      ("FRONT_RIGHT_DOOR", "METER_CLUSTER", 0),
      ("FRONT_LEFT_DOOR", "METER_CLUSTER", 0),
      ("STEER_ANGLE_2", "STEER_MODULE_2", 0.),
      ("DRIVER_EPS_TORQUE", "STEER_MODULE_2", 0.),
      ("MAIN_TORQUE", "STEERING_TORQUE", 0.),
      ("GENERIC_TOGGLE", "STALKS", 0),
      ("RIGHT_BLINKER", "STALKS", 0),
      ("LEFT_BLINKER", "STALKS", 0),
      ("LEFT_APPROACH", "BSM", 0),
      ("RIGHT_APPROACH", "BSM", 0),
      ("STANDSTILL_STATE", "ACC_CMD", 0),
      ("SET_BTN", "PCM_BUTTONS", 0),
      ("RES_BTN", "PCM_BUTTONS", 0),
    ]
    checks = []

    # todo: make it such that enforce_checks=True
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)
