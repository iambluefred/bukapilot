#!/usr/bin/env python3
import cereal.messaging as messaging
from selfdrive.boardd.boardd_api_impl import can_list_to_can_capnp
from opendbc.can.packer import CANPacker
from selfdrive.car.perodua.peroduacan import create_can_steer_command
from selfdrive.car.perodua.values import ACC_CAR, CAR, DBC, NOT_CAN_CONTROLLED, BRAKE_SCALE, GAS_SCALE

VEHICLE = "PERODUA MYVI PSD"

def signal_injection():
  pm = messaging.PubMaster(['sendcan'])
  sm = messaging.SubMaster(['testJoystick'])
  frame = 0
  packer = CANPacker(DBC[VEHICLE]['pt'])
  
  while 1:
    can_sends = []
    sm.update()
    frame += 1
    gb = 0
    steer = 0

    if len(sm['testJoystick'].axes) != 0:
      gb = sm['testJoystick'].axes[0]
      steer = sm['testJoystick'].axes[1]

    can_sends.append(create_can_steer_command(packer, steer, 1, (frame/2) % 16))
    pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=True))


if __name__ == "__main__":
  signal_injection()
