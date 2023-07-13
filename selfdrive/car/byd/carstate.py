from cereal import car
from collections import deque
from math import ceil
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.byd.values import DBC, CAR
from time import time

from common.features import Features

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["DRIVE_STATE"]['GEAR']
    self.set_distance_values = can_define.dv['ACC_HUD']['SET_DISTANCE']
    self.is_cruise_latch = False
    self.acc_req = False
    self.hand_on_wheel_warning = False
    self.is_icc_on = False
    self.prev_angle = 0

    f = Features()
    self.mads = f.has("StockAcc")

    self.stock_lks_settings = 0

  def update(self, cp):
    ret = car.CarState.new_message()

    self.stock_lks_settings = 0 # Todo

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_FR'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_BL'],
      cp.vl["WHEEL_SPEED"]['WHEELSPEED_BR'],
    )
    ret.vEgoRaw = mean([ret.wheelSpeeds.rr, ret.wheelSpeeds.rl, ret.wheelSpeeds.fr, ret.wheelSpeeds.fl])

    # unfiltered speed from CAN sensors
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    # safety checks to engage
    can_gear = int(cp.vl["DRIVE_STATE"]['GEAR'])

    # Todo
    ret.doorOpen = 0 #any([cp.vl["DOOR_LEFT_SIDE"]['BACK_LEFT_DOOR'],
                   #  cp.vl["DOOR_LEFT_SIDE"]['FRONT_LEFT_DOOR'],
                   #  cp.vl["DOOR_RIGHT_SIDE"]['BACK_RIGHT_DOOR'],
                   #  cp.vl["DOOR_RIGHT_SIDE"]['FRONT_RIGHT_DOOR']])

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]['SEATBELT_DRIVER'] == 0
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.brakeHoldActive = 0 #Todo bool(cp.vl["PARKING_BRAKE"]["CAR_ON_HOLD"])

    disengage = ret.doorOpen or ret.seatbeltUnlatched or ret.brakeHoldActive
    if disengage:
      self.is_cruise_latch = False

    # gas pedal
    ret.gas = cp.vl["PEDAL"]['GAS_PEDAL']
    ret.gasPressed = ret.gas > 0.01

    # brake pedal
    ret.brake = cp.vl["PEDAL"]['BRAKE_PEDAL']
    if self.mads:
      ret.brakePressed = False
    else:
      ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"])

    # steer
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]['STEER_ANGLE']
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]['MAIN_TORQUE'] * steer_dir
    ret.steeringTorqueEps = cp.vl["STEERING_MODULE_2"]['DRIVER_EPS_TORQUE'] * steer_dir
    ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 5)
    ret.steerWarning = False
    ret.steerError = False
    self.hand_on_wheel_warning = 0 # Todo bool(cp.vl["ADAS_LKAS"]["HAND_ON_WHEEL_WARNING"])
    self.is_icc_on = 0 # Todo  bool(cp.vl["PCM_BUTTONS"]["ICC_ON"])

    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    # Todo: get the real value
    ret.stockAeb = False
    ret.stockFcw = False

    self.acc_req = 0 # Todo bool(cp.vl["ACC_CMD"]["ACC_REQ"])
    ret.cruiseState.available = any([cp.vl["ACC_HUD"]["ACC_ON1"], cp.vl["ACC_HUD"]["ACC_ON2"]]) # Todo, need two?

    distance_val = int(cp.vl["ACC_HUD"]['SET_DISTANCE'])
    ret.cruiseState.setDistance = self.parse_set_distance(self.set_distance_values.get(distance_val, None))

    # engage and disengage logic
    #if cp.vl["PCM_BUTTONS"]["ACC_SET"] == 0 and ret.brakePressed:
    #  self.is_cruise_latch = False

    #if cp.vl["PCM_BUTTONS"]["ACC_SET"] != 0 and not ret.brakePressed:
    #  self.is_cruise_latch = True

    # set speed in range of 30 - 130kmh only
    self.cruise_speed = int(cp.vl["ACC_HUD"]['SET_SPEED_MAYBE']) * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = self.cruise_speed
    ret.cruiseState.speed = ret.cruiseState.speedCluster / HUD_MULTIPLIER
    ret.cruiseState.standstill = 0 # Todo bool(cp.vl["ACC_CMD"]["STANDSTILL2"])
    ret.cruiseState.nonAdaptive = False

    if not ret.cruiseState.available:
      self.is_cruise_latch = False

    if not self.mads:
      if ret.brakePressed or (not self.acc_req and not ret.cruiseState.standstill):
        self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch

    # button presses
    ret.leftBlinker = bool(cp.vl["STALK"]["LEFT_BLINKER"])
    ret.rightBlinker = 0 bool(cp.vl["STALK"]["RIGHT_BLINKER"])
    ret.genericToggle = 0 # Todo bool(cp.vl["LEFT_STALK"]["GENERIC_TOGGLE"])

    ret.espDisabled = 0 # Todo bool(cp.vl["PARKING_BRAKE"]["ESC_ON"]) != 1

    # blindspot sensors
    if self.CP.enableBsm:
      # used for lane change so its okay for the chime to work on both side.
      ret.leftBlindspot = bool(cp.vl["BSM"]["LEFT_APPROACH"])
      ret.rightBlindspot = 0 # Todo bool(cp.vl["BSM"]["RIGHT_APPROACH"])


    return ret


  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address, default
      ("GEAR", "DRIVE_STATE", 1),
      ("BRAKE_PRESSED", "DRIVE_STATE", 1),
      ("SET_DISTANCE", "ACC_HUD", 1),
      ("SET_SPEED_MAYBE", "ACC_HUD", 1),
      ("ACC_ON1", "ACC_HUD", 1),
      ("ACC_ON2", "ACC_HUD", 1),
      ("WHEELSPEED_FR", "WHEEL_SPEED", 1),
      ("WHEELSPEED_FL", "WHEEL_SPEED", 1),
      ("WHEELSPEED_BR", "WHEEL_SPEED", 1),
      ("WHEELSPEED_BL", "WHEEL_SPEED", 1),
      ("GAS_PEDAL", "PEDAL", 1),
      ("BRAKE_PEDAL", "PEDAL", 1),
      ("SEATBELT_DRIVER", "SEATBELT", 1),
      ("STEER_ANGLE", "STEERING_MODULE", 1),
      ("DRIVER_EPS_TORQUE", "STEERING_MODULE_2", 1),
      ("MAIN_TORQUE", "STEERING_TORQUE", 1),
      ("RIGHT_BLINKER", "STALK", 1),
      ("LEFT_BLINKER", "STALK", 1),
      ("LEFT_APPROACH", "BSM", 1),
    ]
    checks = []

    # todo: make it such that enforce_checks=True
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0, enforce_checks=False)
