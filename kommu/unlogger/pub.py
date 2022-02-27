#!/usr/bin/env python3
import cereal.messaging as messaging
from common.realtime import Ratekeeper
import time

def main():

    # cereal
    pm = messaging.PubMaster(['ubloxRaw'])
    dat = messaging.new_message('ubloxRaw', 1)
    while True:
      dat.ubloxRaw = "$PSTMDRSTATE,83474129,2.43424,1.53425,180,19,2.1211,1.1111,3.22313,,1.332323,12.1*1E\r\n"
      pm.send('ubloxRaw', dat)
      time.sleep(0.1)


if __name__ == "__main__":
    main()
