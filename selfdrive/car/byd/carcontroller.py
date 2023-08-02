from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.byd.bydcan import create_can_steer_command, create_accel_command, send_buttons
from selfdrive.car.byd.values import CAR, DBC
from selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from opendbc.can.packer import CANPacker
from common.numpy_fast import clip, interp
from common.realtime import DT_CTRL
from common.params import Params
import cereal.messaging as messaging

from common.features import Features

def apply_byd_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, LIMITS):

  # limits due to driver torque
  driver_max_torque = LIMITS.STEER_MAX + driver_torque * 30
  driver_min_torque = -LIMITS.STEER_MAX + driver_torque * 30
  max_steer_allowed = max(min(LIMITS.STEER_MAX, driver_max_torque), 0)
  min_steer_allowed = min(max(-LIMITS.STEER_MAX, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))

class CarControllerParams():
  def __init__(self, CP):

    self.STEER_MAX = CP.lateralParams.torqueV[0]
    assert(len(CP.lateralParams.torqueV) == 1)

    # for torque limit calculation
    self.STEER_DELTA_UP = 20                      # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 30                    # torque decrease per refresh

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.disable_radar = Params().get_bool("DisableRadar")
    self.num_cruise_btn_sent = 0
    self.random_counter = 0

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw):
    can_sends = []

    # steer
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_angle = actuators.steeringAngleDeg
    # TODO
    #apply_steer = apply_byd_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, self.params)
    apply_steer = apply_byd_steer_torque_limits(new_steer, self.last_steer, 0, self.params)

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)

    ts = frame * DT_CTRL

    if (frame % 1000) == 0:
      self.random_counter = (self.random_counter + 1) & 0xF

    # BYD CAN controlled lateral running at 50hz
    if (frame % 2) == 0:
      if CS.out.leftBlinker and False:
        can_sends.append(create_can_steer_command(self.packer, 0, enabled, False, (frame/2) % 16, 0, self.random_counter))
        #can_sends.append(create_can_steer_command(self.packer, 10, enabled, False, (frame/2) % 16, 2773))
      else:
        can_sends.append(create_can_steer_command(self.packer, apply_angle, enabled, False, (frame/2) % 16, 0, self.random_counter))

      if CS.out.genericToggle:
        # accel
        can_sends.append(create_accel_command(self.packer, 40, 1, (frame/2) % 16, self.randon_counter))
        # decel
        # can_sends.append(create_accel_command(self.packer, -40, 1, (frame/2) % 16))
      else:
        can_sends.append(create_accel_command(self.packer, 0, 0, (frame/2) % 16, self.random_counter))
      #  a = list(create_accel_command(self.packer, 0, 0, (frame/2) % 16)[2])
      #  b = [hex(i) for i in a]
      #  print(b)


      #can_sends.append(create_hud(self.packer, apply_steer, enabled, ldw, rlane_visible, llane_visible))
      #can_sends.append(create_lead_detect(self.packer, lead_visible, enabled))
      #if CS.out.genericToggle:
      #  fake_enable = True
      #else:
      #  fake_enable = False
      #can_sends.append(create_acc_cmd(self.packer, actuators.accel, fake_enable, (frame/2) % 16))

    #if CS.out.standstill and enabled and (frame % 50 == 0):
      # Spam resume button to resume from standstill at max freq of 10 Hz.
      #if not self.mads or CS.acc_req:
        #can_sends.append(send_buttons(self.packer, frame % 16, False))

    if CS.out.standstill and enabled and (frame % 50 == 0):
      # Spam resume button to resume from standstill at max freq of 20 Hz.
      can_sends.append(send_buttons(self.packer, frame % 16))

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    return new_actuators, can_sends
