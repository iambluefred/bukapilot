#!/usr/bin/env python3
from panda import Panda
import time

# list format [addr, msg, bus, freq]
tx = [[0xaa, b"\xaa" * 4, 0, 10 ], [0xaf, b"\xaa" * 4, 0, 10 ],[0xab, b"\xaa", 0, 1]]
spam_time_s = 1 # don't inject more than 5 seconds to prevent dangerous lockouts

if __name__ == "__main__":
  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  start_time = time.time()

  # get all the frequencies and group the unique freqs
  freqs = [i[3] for i in tx]
  freqs = sorted(freqs, reverse=True)
  freqs = list(dict.fromkeys(freqs))

  # group tx messages into sending order
  tx_msgs = []
  for i, f in enumerate(freqs):
    for m in tx:
      if f == m[3]:
        tx_msgs[i].append(m)

  # get the sending dt
  send_time_s = [1/f for f in freqs]
  send_time_subtracted = []
  for i in range(len(send_time_s)):
    send_time_subtracted.append(send_time_s[i] - sum(send_time_s[:i]))

  print("Spamming all buses...")
  while True:

    if time.time() - start_time > spam_time_s:
      break

    for i, t in enumerate(send_time_s):
      for m in tx_msgs[i]:
        p.can_send(m[0], m[1], m[2])
      time.sleep(t)


  p.set_safety_mode(Panda.SAFETY_SILENT)
