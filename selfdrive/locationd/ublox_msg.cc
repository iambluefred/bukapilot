#include "ublox_msg.h"

#include <unistd.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <unordered_map>

#include "selfdrive/common/swaglog.h"
//const double gpsPi = 3.1415926535898;
#define UBLOX_MSG_SIZE(hdr) (*(uint16_t *)&hdr[4])

//inline static bool bit_to_bool(uint8_t val, int shifts) {
//  return (bool)(val & (1 << shifts));
//}

inline int UbloxMsgParser::needed_bytes(const uint8_t *incoming_data) {
  int needed_bytes = 0;

  while(true) {
    if(*(incoming_data++) != '\r') {
      needed_bytes++;

      // Limit the bytes into buffer
      if(needed_bytes >= 65535) {
        return -1;
      }
    }
    else {
      if((*(incoming_data++) == '\n')) {
        needed_bytes+=3;
        break;
      }
    }
  }
  return needed_bytes;
}

bool UbloxMsgParser::add_data(const uint8_t *incoming_data, uint32_t incoming_data_len, size_t &bytes_consumed) {

  int needed = needed_bytes(incoming_data);
  LOGE("Needed: %d", needed);

  if(needed > 0) {
    bytes_consumed = std::min((uint32_t)needed, incoming_data_len );
    // Add data to buffer
    memcpy(msg_parse_buf + bytes_in_parse_buf, incoming_data, bytes_consumed);
    bytes_in_parse_buf += bytes_consumed;
  } else {
    bytes_consumed = incoming_data_len;
  }

  // There is redundant data at the end of buffer, reset the buffer.
  if(needed == -1) {
    bytes_in_parse_buf = 0;
    return false;
  }
  LOGE("Bytes in buf: %s", msg_parse_buf);

  return true;
}


std::pair<std::string, kj::Array<capnp::word>> UbloxMsgParser::gen_msg() {
  std::string dat = data();
  kaitai::kstream stream(dat);

  quectel_t quectel_msg(&stream);
  auto message = quectel_msg.messages();
  auto body = message->dat();
  std::string sentence = message->sentence();

  if (sentence == "TMDRSTATE"){
    return {"gpsLocationExternal", gen_gps_loc_ext(static_cast<quectel_t::pstmdrstate_t*>(body))};
  }
  else {
    LOGE("Unknown msg_id %s", sentence.c_str());
    return {"ubloxGnss", kj::Array<capnp::word>()};
  }
}

kj::Array<capnp::word> UbloxMsgParser::gen_gps_loc_ext(quectel_t::pstmdrstate_t *msg) {
  MessageBuilder msg_builder;
  auto gpsLoc = msg_builder.initEvent().initGpsLocationExternal();

  //std::string s_lat = msg->lat();

  gpsLoc.setSource(cereal::GpsLocationData::SensorSource::UBLOX);
  gpsLoc.setFlags(1);
  gpsLoc.setLatitude(1 * 1e-07);
  gpsLoc.setLongitude(1.1 * 1e-07);
  gpsLoc.setAltitude(1.1 * 1e-03);
  gpsLoc.setSpeed(12 * 1e-03);
  gpsLoc.setBearingDeg(1* 1e-5);
  gpsLoc.setAccuracy(1* 1e-03);
  std::tm timeinfo = std::tm();
  timeinfo.tm_year = 2020 - 1900;
  timeinfo.tm_mon = 12 - 1;
  timeinfo.tm_mday = 1;
  timeinfo.tm_hour = 1;
  timeinfo.tm_min = 1;
  timeinfo.tm_sec = 1;

  std::time_t utc_tt = timegm(&timeinfo);
  gpsLoc.setTimestamp(utc_tt * 1e+03 + 1* 1e-06);
  float f[] = { 1 * 1e-03f, 1 * 1e-03f, 1 * 1e-03f };
  gpsLoc.setVNED(f);
  gpsLoc.setVerticalAccuracy(1 * 1e-03);
  gpsLoc.setSpeedAccuracy(1 * 1e-03);
  gpsLoc.setBearingAccuracyDeg(1* 1e-05);

  return capnp::messageToFlatArray(msg_builder);
}
