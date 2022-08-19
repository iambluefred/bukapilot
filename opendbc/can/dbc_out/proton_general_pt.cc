#include "common_dbc.h"

namespace {

const Signal sigs_257[] = {
    {
      .name = "BRAKE_ENGAGE",
      .b1 = 63,
      .b2 = 1,
      .bo = 0,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_277[] = {
    {
      .name = "GAS",
      .b1 = 33,
      .b2 = 7,
      .bo = 24,
      .is_signed = false,
      .factor = 0.00999,
      .offset = -0.08,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "GEAR",
      .b1 = 54,
      .b2 = 2,
      .bo = 8,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_528[] = {
    {
      .name = "WHEELSPEED_FRONT",
      .b1 = 0,
      .b2 = 16,
      .bo = 48,
      .is_signed = false,
      .factor = 0.0088,
      .offset = -0.0704,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "WHEELSPEED_BACK",
      .b1 = 16,
      .b2 = 16,
      .bo = 32,
      .is_signed = false,
      .factor = 0.0011,
      .offset = -0.0704,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};
const Signal sigs_1296[] = {
    {
      .name = "HANDBRAKE",
      .b1 = 47,
      .b2 = 1,
      .bo = 16,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "DOOR",
      .b1 = 46,
      .b2 = 1,
      .bo = 17,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "LIGHT",
      .b1 = 45,
      .b2 = 1,
      .bo = 18,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
    {
      .name = "SEATBELT",
      .b1 = 55,
      .b2 = 1,
      .bo = 8,
      .is_signed = false,
      .factor = 1,
      .offset = 0,
      .is_little_endian = false,
      .type = SignalType::DEFAULT,
    },
};

const Msg msgs[] = {
  {
    .name = "BRAKE_PEDAL",
    .address = 0x101,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_257),
    .sigs = sigs_257,
  },
  {
    .name = "TRANSMISSION",
    .address = 0x115,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_277),
    .sigs = sigs_277,
  },
  {
    .name = "WHEELSPEED",
    .address = 0x210,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_528),
    .sigs = sigs_528,
  },
  {
    .name = "METER_CLUSTER",
    .address = 0x510,
    .size = 8,
    .num_sigs = ARRAYSIZE(sigs_1296),
    .sigs = sigs_1296,
  },
};

const Val vals[] = {
    {
      .name = "GEAR",
      .address = 0x115,
      .def_val = "1 N 2 D 3 S 0 P",
      .sigs = sigs_277,
    },
};

}

const DBC proton_general_pt = {
  .name = "proton_general_pt",
  .num_msgs = ARRAYSIZE(msgs),
  .msgs = msgs,
  .vals = vals,
  .num_vals = ARRAYSIZE(vals),
};

dbc_init(proton_general_pt)