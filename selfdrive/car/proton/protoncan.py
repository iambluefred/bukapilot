from common.numpy_fast import clip, interp
from cereal import car
from selfdrive.config import Conversions as CV

SetDistance = car.CarState.CruiseState.SetDistance

# reference: http://sunshine2k.de/articles/coding/crc/understanding_crc.html#ch3
crc8_lut_8h2f = []

def init_lut_crc8_8h2f():
  poly = 0x2F

  for i in range(256):
    crc = i
    for j in range(8):
      if ((crc & 0x80) != 0):
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc <<= 1
    crc8_lut_8h2f.append(crc)

def get_crc8_8h2f(dat):
  crc = 0xFF    # initial value for crc8_8h2f
  for i in range(len(dat)):
    crc ^= dat[i]
    crc = crc8_lut_8h2f[crc]

  return crc ^ 0xFF

def compute_set_distance(state):
  if state == SetDistance.aggresive:
    return 2
  elif state == SetDistance.normal:
    return 1
  else:
    return 0

def create_can_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""
  values = {
    "LKAS_ENGAGED1": steer_req,
    "LKAS_ENGAGED2": steer_req,
    "UNKNOWN": 1,
    "STEER_CMD": steer if steer_req else 0,
    # steer positive is clockwise
    "STEER_DIR": 1 if (steer >= 0 or not steer_req) else 0,
    "COUNTER": raw_cnt,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
    "SET_ME_1_3": 1,
    "SET_ME_48": 0x48,
    "HAND_ON_WHEEL_WARNING": 0,
    "WHEEL_WARNING_CHIME": 0,
  }

  dat = packer.make_can_msg("ADAS_LKAS", 0, values)[2]
  crc = get_crc8_8h2f(dat[:-1])
  values["CHECKSUM"] = 0

  return packer.make_can_msg("ADAS_LKAS", 0, values)

def create_hud(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""
  values = {
    "LANE_DEPARTURE_WARNING_RIGHT": 0,
    "LANE_DEPARTURE_WARNING_LEFT": 0,
    "LEFT_LANE_VISIBLE_DISENGAGE": 0,
    "RIGHT_LANE_VISIBLE_DISENGAGE": 0,
    "STEER_REQ_RIGHT": 1 if steer_req else 0,
    "STEER_REQ_LEFT": 1 if steer_req else 0,
    "STEER_REQ_MAJOR": 1 if steer_req else 0,
    "STEER_CMD": 0x91 if steer_req else 0x4b,
    "NEW_SIGNAL_2": 0x3f if steer_req else 0x3f,
    "NEW_SIGNAL_1": 0xaa if steer_req else 0x3d,
  }

  dat = packer.make_can_msg("LKAS", 0, values)[2]
  return packer.make_can_msg("LKAS", 0, values)

def create_car_detect(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""
  values = {
    "LEFT_LANE_CAR_DIST": 0,
    "NEW_SIGNAL_1": 0x7e,
    "NEW_SIGNAL_2": 0x7e,
    "LEFT_LANE_CAR_EXIST": 0,
    "RIGHT_LANE_CAR_DIST": 0,
    "RIGHT_LANE_CAR_EXIST": 0,
  }

  dat = packer.make_can_msg("ADAS_CAR_DETECT", 0, values)[2]
  return packer.make_can_msg("ADAS_CAR_DETECT", 0, values)

def create_lead_detect(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""
  values = {
    "LEAD_DISTANCE": 0,
    "NEW_SIGNAL_1": 0x7f,
    "NEW_SIGNAL_2": 0x7e,
    "IS_LEAD2": 0,
    "IS_LEAD1": 0,
    "LEAD_TOO_NEAR": 0,
  }

  dat = packer.make_can_msg("ADAS_LEAD_DETECT", 0, values)[2]
  return packer.make_can_msg("ADAS_LEAD_DETECT", 0, values)

def create_heartbeat(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""
  values = {
    "COUNTER": raw_cnt,
    "NEW_SIGNAL_1": 1537,
    "NEW_SIGNAL_2": 32773,
    "NEW_SIGNAL_3": 128,
  }

  dat = packer.make_can_msg("ADAS_HEARTBEAT", 0, values)[2]
  crc = get_crc8_8h2f(dat[:-1])
  values["CHECKSUM"] = crc

  return packer.make_can_msg("ADAS_HEARTBEAT", 0, values)

def create_pcm(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Perodua LKA Steer Command."""
  values = {
    "ACC_SET_SPEED": 0x23 if steer_req else 0,
    "SET_DISTANCE": 1 if steer_req else 0,
    "NEW_SIGNAL_1": 3,
    "ACC_SET": 1 if steer_req else 0,
    "COUNTER": raw_cnt,
    "ACC_ON_OFF_BUTTON": 1,
  }

  dat = packer.make_can_msg("PCM_BUTTONS", 0, values)[2]
  crc = get_crc8_8h2f(dat[:-1])
  values["CHECKSUM"] = crc

  return packer.make_can_msg("PCM_BUTTONS", 0, values)



init_lut_crc8_8h2f()
