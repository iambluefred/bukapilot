from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.perodua.peroduacan import create_steer_command, perodua_create_gas_command, \
                                             perodua_aeb_warning, create_can_steer_command, \
                                             perodua_create_accel_command, \
                                             perodua_create_brake_command, perodua_create_hud
from selfdrive.car.perodua.values import ACC_CAR, CAR, DBC, NOT_CAN_CONTROLLED, BRAKE_SCALE, GAS_SCALE
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
import cereal.messaging as messaging

from bisect import bisect_left

from common.features import Features

BRAKE_THRESHOLD = 0.01
BRAKE_MAG = [BRAKE_THRESHOLD,.32,.46,.61,.76,.90,1.06,1.22,1.36,1.50,1.66,1.80,1.94,2.10,2.26,2.41,4.0]
PUMP_VALS = [0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6]

def apply_acttr_steer_torque_limits(apply_torque, apply_torque_last, LIMITS):
  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

def compute_gb(accel):
  gb = float(accel) / 4.0
  return clip(gb, 0.0, 1.0), clip(-gb, 0.0, 1.0)

def psd_brake(apply_brake, last_pump_start_ts, ts):
  saturated = False

  # reversed engineered from Ativa stock braking
  # this is necessary for noiseless pump braking
  pump = PUMP_VALS[bisect_left(BRAKE_MAG, apply_brake)]

  if apply_brake >= BRAKE_THRESHOLD:
    brake_req = 1
  else:
    last_pump_start_ts = ts
    brake_req = 0

  # todo : reset pump timer if:
  # - we are applying steady state brakes and we haven't been running the pump
  #   for more than 3s (to prevent pressure bleeding)
  if (ts - last_pump_start_ts > 5 and apply_brake > BRAKE_THRESHOLD):
    saturated = True

  return pump, last_pump_start_ts, brake_req, saturated

class CarControllerParams():
  def __init__(self, CP):

    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV

    # for torque limit calculation
    if CP.carFingerprint in NOT_CAN_CONTROLLED:
      self.STEER_DELTA_UP = 20                      # torque increase per refresh, 0.8s to max
      self.STEER_DELTA_DOWN = 30                    # torque decrease per refresh
    else:
      self.STEER_DELTA_UP = 10
      self.STEER_DELTA_DOWN = 30

    self.STEER_REDUCE_FACTOR = 1000                 # how much to divide the steer when reducing fighting torque
    self.GAS_MAX = 2600                             # KommuActuator dac gas value
    self.GAS_STEP = 2                               # how often we update the longitudinal cmd
    self.BRAKE_ALERT_PERCENT = 30                   # percentage of brake to sound stock AEB alert
    self.ADAS_STEP = 5                              # 100/5 approx ASA frequency of 20 hz

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.last_pump_start_ts = 0.
    self.pump_saturated = False
    self.steer_rate_limited = False
    self.steering_direction = False
    self.brake_pressed = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.brake = 0
    self.brake_scale = BRAKE_SCALE[CP.carFingerprint]
    self.gas_scale = GAS_SCALE[CP.carFingerprint]
    f = Features()
    self.need_clear_engine = f.has("ClearCode")

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw):
    can_sends = []

    # steer
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    new_steer = int(round(actuators.steer * steer_max_interp))
    apply_steer = apply_acttr_steer_torque_limits(new_steer, self.last_steer, self.params)
    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)
    if CS.CP.carFingerprint not in NOT_CAN_CONTROLLED:
      self.steer_rate_limited &= not CS.out.steeringPressed

    # gas, brake
    apply_gas, apply_brake = compute_gb(actuators.accel)
    apply_brake *= self.brake_scale
    if CS.out.gasPressed:
      apply_brake = 0
    apply_gas *= self.gas_scale

    '''
    Perodua vehicles supported by Kommu includes vehicles that does not have stock LKAS and ACC.
    These vehicles are controlled by KommuActuator and is labelled as NOT_CAN_CONTROLLED.
    KommuActuator uses 4 DACs to control the gas and steer of the vehicle. All values reflects the
    value of 12 bit DAC.

    Vehicles that comes with Perodua Smart Drive (PSD), includes stock LKAS and ACC. Note that the
    ACC command is done by giving a set speed so the stock internal controller will obtain the speed.
    '''

    if CS.CP.carFingerprint not in NOT_CAN_CONTROLLED:
      ts = frame * DT_CTRL

      # CAN controlled lateral
      if (frame % 2) == 0:

        # allow stock LDP passthrough
        stockLdw = CS.out.stockAdas.laneDepartureHUD
        if stockLdw:
            apply_steer = -CS.out.stockAdas.ldpSteerV

        steer_req = enabled or stockLdw
        can_sends.append(create_can_steer_command(self.packer, apply_steer, steer_req, (frame/2) % 15))

      # CAN controlled longitudinal
      if (frame % 5) == 0 and CS.CP.safetyConfigs[0].safetyParam == 1:
        # PSD brake logic
        if self.need_clear_engine:
          can_sends.append(make_can_msg(2015, b'\x01\x04\x00\x00\x00\x00\x00\x00', 0))

        pump, self.last_pump_start_ts, brake_req, self.pump_saturated = psd_brake(apply_brake, self.last_pump_start_ts, ts)

        mult = CS.out.vEgo * (apply_gas - apply_brake)
        des_speed = max(0, CS.out.vEgo + mult)
        can_sends.append(perodua_create_accel_command(self.packer, CS.out.cruiseState.speedCluster,
                                                      CS.out.cruiseState.available, enabled, lead_visible,
                                                      des_speed, apply_brake, pump, CS.out.cruiseState.setDistance))
        can_sends.append(perodua_create_brake_command(self.packer, enabled, brake_req, pump, apply_brake, CS.out.stockAeb, (frame/5) % 8))
        can_sends.append(perodua_create_hud(self.packer, CS.out.cruiseState.available, enabled, llane_visible, rlane_visible, ldw, CS.out.stockFcw, CS.out.stockAeb, CS.out.stockAdas.frontDepartureHUD))

    # KommuActuator controls
    else:
      # steer
      reduce_fighting_torque = ((CS.out.vEgo < LANE_CHANGE_SPEED_MIN) and (CS.out.leftBlinker != CS.out.rightBlinker))
      if reduce_fighting_torque:
        apply_steer = apply_steer / self.params.STEER_REDUCE_FACTOR
      self.steering_direction = True if (apply_steer >= 0) else False

      can_sends.append(create_steer_command(self.packer, apply_steer, self.steering_direction, enabled, frame))

      # gas
      if (frame % self.params.GAS_STEP) == 0 and CS.CP.safetyConfigs[0].safetyParam == 1:
        idx = frame // self.params.GAS_STEP

        if CS.CP.enableGasInterceptor:
          can_sends.append(perodua_create_gas_command(self.packer, apply_gas, enabled, idx))

      # brakes, AEB alert for non-braking cars
      if (frame % self.params.ADAS_STEP) == 0:
        self.brake = apply_brake

        if apply_brake > (self.params.BRAKE_ALERT_PERCENT / 100):
          if not self.brake_pressed and lead_visible:
            can_sends.append(perodua_aeb_warning(self.packer))
            self.brake_pressed = True
        else:
          self.brake_pressed = False

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    if CS.out.gasPressed:
      new_actuators.accel = 0.5
    new_actuators.steer = apply_steer / steer_max_interp

    return new_actuators, can_sends
