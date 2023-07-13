from common.numpy_fast import clip, interp
from cereal import car
from selfdrive.config import Conversions as CV

SetDistance = car.CarState.CruiseState.SetDistance

def compute_set_distance(state):
  if state == SetDistance.aggresive:
    return 2
  elif state == SetDistance.normal:
    return 1
  else:
    return 0


def byte_crc4_linear_inverse(byte_list):
  return (-1 * sum(byte_list) + 0x9) & 0xf

def byd_checksum(byte_key, dat):

  second_bytes = [byte & 0xf for byte in dat]
  remainder = sum(second_bytes) >> 4
  second_bytes.append(byte_key >> 4)


  first_bytes = [byte >> 4 for byte in dat]
  first_bytes.append(byte_key & 0xf)

  return (((byte_crc4_linear_inverse(first_bytes) + (-1*remainder + 5)) << 4) + byte_crc4_linear_inverse(second_bytes)) & 0xff

def create_can_steer_command(packer, steer, steer_req, wheel_touch_warning, raw_cnt, stock_lks_settings):

  values = {
    "STEER_CMD": stock_lks_settings,
    "STEER_REQ": 1,
    "STEER_ANGLE": steer,
    "COUNTER": raw_cnt,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "UNKNOWN": 3,
  }

  dat = packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)[2]
  crc = byd_checksum(0xaf, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)

