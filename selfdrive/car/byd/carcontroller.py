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

class CarControllerParams():
  def __init__(self, CP):
    pass

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.last_steer = 0
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint]['pt'])
    self.disable_radar = Params().get_bool("DisableRadar")
    self.steer_rate_limited = False
    self.num_cruise_btn_sent = 0
    self.random_counter = 0

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw):
    can_sends = []

    # steer
    apply_angle = actuators.steeringAngleDeg
    ts = frame * DT_CTRL

    if (frame % 1000) == 0:
      self.random_counter = (self.random_counter + 1) & 0xF
    print(self.random_counter)

    # BYD CAN controlled lateral running at 50hz
    if (frame % 2) == 0:
      if CS.out.genericToggle:
        can_sends.append(create_can_steer_command(self.packer, apply_angle, True, False, (frame/2) % 16, 0, 0xF))
#        if CS.out.leftBlinker:
 #         can_sends.append(create_can_steer_command(self.packer, 30, True, False, (frame/2) % 16, 0, 0xB))
      else:
        can_sends.append(create_can_steer_command(self.packer, apply_angle, False, False, (frame/2) % 16, 0, self.random_counter))

      if CS.out.leftBlinker and False:
        # accel
        can_sends.append(create_accel_command(self.packer, 30, 1, (frame/2) % 16, self.random_counter))
      elif CS.out.rightBlinker and False:
        # decel
        can_sends.append(create_accel_command(self.packer, -30, 1, (frame/2) % 16, self.random_counter))
      else:
        can_sends.append(create_accel_command(self.packer, 0, 0, (frame/2) % 16, self.random_counter))
      #  a = list(create_accel_command(self.packer, 0, 0, (frame/2) % 16)[2])
      #  b = [hex(i) for i in a]
      #  print(b)

    if CS.out.standstill and enabled and (frame % 50 == 0):
      # Spam resume button to resume from standstill at max freq of 20 Hz.
      can_sends.append(send_buttons(self.packer, frame % 16))

    new_actuators = actuators.copy()
    new_actuators.steeringAngleDeg = apply_angle

    return new_actuators, can_sends
