#!/usr/bin/env python3
import datetime
import os

import cereal.messaging as messaging

MIN_DATE = datetime.datetime(year=2021, month=4, day=1)

def set_time(logger):
  sys_time = datetime.datetime.today()
  if sys_time > MIN_DATE:
    logger.info("System time valid")
    return

  sock = messaging.sub_sock("gpsNMEA", timeout=5000)
  success = False
  for i in range(10):
    gps = messaging.recv_sock(sock, wait=True)
    try:
      if (gps.gpsNMEA.nmea[3:6] == "RMC"):
        rmc_list = gps.gpsNMEA.nmea.split(",")
        rmc_date_cmd = f"date -s \"20{rmc_list[9][4:6]}-{rmc_list[9][2:4]}-{rmc_list[9][0:2]}\""
        rmc_time_cmd = f"date +%T -s \"{rmc_list[1][0:2]}:{rmc_list[1][2:4]}:{rmc_list[1][4:6]}\""
        os.system(f"{rmc_date_cmd} && {rmc_time_cmd}")
        success = True
        break
    except AttributeError:
      logger.info("RMC not found, datetime not set, retying in awhile")
      break
  if not success:
    logger.warn("Failed to fetch time from GPS")

if __name__ == "__main__":
  import logging
  logging.basicConfig(level=logging.INFO)

  set_time(logging)
