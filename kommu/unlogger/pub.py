#!/usr/bin/env python3
import cereal.messaging as messaging
from common.realtime import Ratekeeper
import time

def main():

    # cereal
    pm = messaging.PubMaster(['ubloxRaw'])
    dat = messaging.new_message('ubloxRaw', 1)
    while True:
      #dat.ubloxRaw = "$PSTMDRSENMSG,30,1385082689,16258,-336,2226*07\r\n"
      dat.ubloxRaw = "$PSTMDRSENMSG,30,1385082689,16258,-336,2226*07\r\n$PSTMDRSENMSG,31,1385082689,119,-137,-103*1E\r\n"
      pm.send('ubloxRaw', dat)
      time.sleep(0.1)


if __name__ == "__main__":
    main()
