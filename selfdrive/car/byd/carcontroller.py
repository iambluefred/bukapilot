from cereal import car
from selfdrive.car import make_can_msg
from selfdrive.car.byd.bydcan import create_can_steer_command
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

    self.STEER_MAX = CP.lateralParams.torqueV[0]
    # make sure Proton only has one max steer torque value
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

    f = Features()
    self.mads = f.has("StockAcc")

  def update(self, enabled, CS, frame, actuators, lead_visible, rlane_visible, llane_visible, pcm_cancel, ldw):
    can_sends = []

    # steer
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    # TODO
    apply_steer = new_steer #apply_byd_steer_torque_limits(new_steer, self.last_steer, 0, self.params)

    self.steer_rate_limited = (new_steer != apply_steer) and (apply_steer != 0)

    ts = frame * DT_CTRL

    # BYD CAN controlled lateral running at 50hz
    if (frame % 2) == 0:
      if CS.out.leftBlinker:
        can_sends.append(create_can_steer_command(self.packer, 100, enabled, False, (frame/2) % 16, -5000))
      else:
        can_sends.append(create_can_steer_command(self.packer, 0, enabled, False, (frame/2) % 16, 0))

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

    self.last_steer = apply_steer
    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX

    return new_actuators, can_sends
