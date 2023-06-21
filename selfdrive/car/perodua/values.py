# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car

from collections import defaultdict
Ecu = car.CarParams.Ecu

HUD_MULTIPLIER = 1.04

class CAR:
  ALZA = "PERODUA ALZA"
  ARUZ = "PERODUA ARUZ"
  ATIVA = "PERODUA ATIVA"
  AXIA = "PERODUA AXIA"
  BEZZA = "PERODUA BEZZA"
  MYVI = "PERODUA MYVI"
  MYVI_PSD = "PERODUA MYVI PSD"

FINGERPRINTS = {
  CAR.AXIA: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 71: 8, 72: 5, 73: 6, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 384: 4, 513: 6, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1267: 8, 1552: 4, 1584: 6, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.ARUZ: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 69: 7, 71: 8, 72: 5, 73: 6, 74: 7, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 355: 7, 384: 4, 513: 8, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 867: 4, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1267: 8, 1552: 4, 1584: 8, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.ALZA: [{
    160: 5, 161: 8, 164: 8, 165: 4, 308: 6, 385: 3, 398: 8, 399: 8, 400: 8, 409: 8, 410: 8, 416: 8, 417: 7, 427: 8, 429: 8, 448: 4, 449: 8, 464: 8, 496: 5, 516: 8, 520: 6, 524: 6, 608: 8, 609: 8, 624: 8, 625: 8, 627: 8, 628: 8, 636: 6, 682: 2, 752: 8, 834: 8, 848: 5, 856: 8, 857: 4, 900: 4, 912: 4, 913: 4, 976: 5, 980: 8, 983: 8, 1012: 7, 1032: 8, 1033: 8, 1034: 8, 1088: 8, 1090: 8, 1100: 8, 1152: 8, 1160: 4, 1162: 8, 1163: 8, 1164: 8, 1168: 8, 1176: 3, 1188: 8, 1200: 3, 1204: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1248: 8, 1252: 8, 1267: 8, 1270: 8, 1312: 8, 1408: 8, 1409: 8, 1410: 8, 1416: 8, 1417: 8, 1418: 8, 1434: 8, 1435: 8, 1542: 2, 1590: 8, 1792: 8, 1798: 3, 1799: 8, 1810: 8, 1813: 8, 1840: 8, 1856: 8, 1858: 8, 1859: 8, 1862: 8, 1872: 8, 1879: 8, 1888: 8, 1892: 8, 1927: 8, 1937: 8, 1945: 8, 1953: 8, 1961: 8, 1968: 8, 1976: 8, 1988: 8, 2000: 8, 2001: 8, 2004: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8, 2025: 8
  },
  {
    160: 5, 161: 8, 164: 8, 165: 4, 204: 20, 308: 6, 385: 3, 398: 8, 399: 8, 400: 8, 409: 8, 410: 8, 416: 8, 417: 7, 424: 4, 427: 8, 429: 8, 448: 4, 449: 8, 464: 8, 496: 5, 516: 8, 520: 6, 524: 6, 608: 8, 609: 8, 624: 8, 625: 8, 627: 8, 628: 8, 636: 6, 640: 6, 682: 2, 752: 8, 834: 8, 848: 5, 856: 8, 857: 4, 900: 4, 912: 4, 913: 4, 976: 5, 980: 8, 1012: 7, 1032: 8, 1033: 8, 1034: 8, 1088: 8, 1090: 8, 1100: 8, 1152: 8, 1160: 4, 1162: 8, 1163: 8, 1164: 8, 1168: 8, 1176: 3, 1188: 8, 1200: 3, 1204: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1244: 8, 1245: 8, 1247: 8, 1248: 8, 1267: 8, 1270: 8, 1312: 8, 1408: 8, 1409: 8, 1410: 8, 1416: 8, 1417: 8, 1418: 8, 1434: 8, 1435: 8, 1542: 2, 1590: 8, 1792: 8, 1798: 3, 1799: 8, 1810: 8, 1813: 8, 1840: 8, 1856: 8, 1858: 8, 1859: 8, 1862: 8, 1872: 8, 1879: 8, 1888: 8, 1892: 8, 1927: 8, 1937: 8, 1945: 8, 1953: 8, 1961: 8, 1968: 8, 1976: 8, 1988: 8, 1996: 8, 2000: 8, 2001: 8, 2004: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8, 2025: 8
  }],
  CAR.ATIVA: [{
    160: 5, 161: 8, 164: 8, 165: 4, 308: 6, 398: 8, 399: 8, 400: 8, 409: 8, 410: 8, 416: 8, 417: 7, 427: 8, 429: 8, 448: 4, 449: 8, 464: 8, 496: 5, 516: 8, 520: 6, 524: 6, 608: 8, 609: 8, 624: 8, 625: 8, 627: 8, 628: 8, 636: 6, 752: 8, 834: 8, 848: 5, 856: 8, 857: 4, 900: 4, 912: 4, 913: 4, 976: 5, 980: 8, 1012: 7, 1032: 8, 1033: 8, 1034: 8, 1088: 8, 1090: 8, 1100: 8, 1152: 8, 1160: 4, 1162: 8, 1163: 8, 1164: 8, 1168: 8, 1176: 3, 1188: 8, 1200: 3, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1248: 8, 1267: 8, 1270: 8, 1312: 8, 1408: 8, 1409: 8, 1410: 8, 1416: 8, 1417: 8, 1418: 8, 1434: 8, 1435: 8
  },
  {
    160: 5, 161: 8, 164: 8, 165: 4, 308: 6, 385: 3, 398: 8, 399: 8, 400: 8, 409: 8, 410: 8, 416: 8, 417: 7, 427: 8, 429: 8, 448: 4, 449: 8, 464: 8, 496: 5, 516: 8, 520: 6, 524: 6, 608: 8, 609: 8, 624: 8, 625: 8, 627: 8, 628: 8, 636: 6, 682: 2, 752: 8, 834: 8, 848: 5, 856: 8, 857: 4, 900: 4, 912: 4, 913: 4, 976: 5, 980: 8, 1012: 7, 1032: 8, 1033: 8, 1034: 8, 1088: 8, 1090: 8, 1100: 8, 1152: 8, 1160: 4, 1162: 8, 1163: 8, 1164: 8, 1168: 8, 1176: 3, 1188: 8, 1200: 3, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1248: 8, 1267: 8, 1270: 8, 1312: 8, 1408: 8, 1409: 8, 1410: 8, 1416: 8, 1417: 8, 1418: 8, 1434: 8, 1435: 8, 1542: 2, 1590: 8, 1792: 8, 1798: 3, 1799: 8, 1810: 8, 1813: 8, 1840: 8, 1856: 8, 1858: 8, 1859: 8, 1862: 8, 1872: 8, 1879: 8, 1888: 8, 1892: 8, 1927: 8, 1937: 8, 1945: 8, 1953: 8, 1961: 8, 1968: 8, 1976: 8, 1988: 8, 2000: 8, 2001: 8, 2004: 8, 2015: 8, 2016: 8, 2017: 8, 2024: 8, 2025: 8
  }],
  CAR.BEZZA: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 69: 7, 71: 8, 72: 5, 73: 6, 74: 7, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 355: 7, 384: 4, 513: 8, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 867: 4, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1267: 8, 1552: 4, 1584: 6, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.MYVI: [{
    32: 4, 33: 8, 36: 4, 37: 8, 40: 3, 69: 7, 71: 8, 72: 5, 73: 6, 74: 7, 76: 7, 77: 7, 85: 7, 88: 5, 102: 6, 107: 4, 122: 6, 128: 5, 186: 3, 355: 7, 384: 4, 513: 6, 592: 8, 593: 8, 679: 8, 680: 8, 681: 7, 800: 8, 802: 7, 867: 4, 945: 2, 977: 3, 1088: 8, 1090: 8, 1100: 8, 1162: 8, 1163: 8, 1164: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1267: 8, 1552: 4, 1584: 8, 1586: 7, 1588: 8, 1595: 8, 1616: 8
  }],
  CAR.MYVI_PSD: [{
    160: 5, 161: 8, 164: 8, 165: 4, 308: 6, 385: 3, 398: 8, 399: 8, 400: 8, 405: 5, 409: 8, 410: 8, 416: 8, 417: 7, 427: 8, 429: 8, 448: 4, 449: 8, 464: 8, 496: 5, 516: 8, 520: 6, 524: 6, 608: 8, 609: 8, 624: 8, 625: 8, 627: 8, 628: 8, 682: 2, 752: 8, 834: 8, 848: 5, 856: 8, 857: 4, 900: 4, 976: 5, 980: 8, 1012: 7, 1032: 8, 1033: 8, 1034: 8, 1088: 8, 1090: 8, 1100: 8, 1152: 8, 1160: 4, 1162: 8, 1163: 8, 1164: 8, 1168: 8, 1176: 3, 1188: 8, 1200: 3, 1204: 8, 1217: 8, 1218: 8, 1219: 8, 1224: 8, 1245: 8, 1247: 8, 1248: 8, 1267: 8, 1312: 8, 1329: 8, 1408: 8, 1409: 8, 1410: 8, 1416: 8, 1417: 8, 1418: 8, 1434: 8, 1435: 8, 1542: 2, 1798: 3
  }],
}

ECU_FINGERPRINT = {
  # ASA Camera CAN fingerprint
  Ecu.fwdCamera: [679, 680, 681, 1267]
}

DBC = {
  CAR.ARUZ: dbc_dict('perodua_general_pt', None),
  CAR.AXIA: dbc_dict('perodua_general_pt', None),
  CAR.BEZZA: dbc_dict('perodua_general_pt', None),
  CAR.MYVI: dbc_dict('perodua_general_pt', None),
  CAR.ATIVA: dbc_dict('perodua_psd_pt', None),
  CAR.ALZA: dbc_dict('perodua_psd_pt', None),
  CAR.MYVI_PSD: dbc_dict('perodua_psd_pt', None),
}

BRAKE_SCALE = defaultdict(lambda: 1, {CAR.ATIVA: 3.0, CAR.MYVI_PSD: 3.0, CAR.ALZA: 2.6})
GAS_SCALE = defaultdict(lambda: 2600, {CAR.ATIVA: 0.4, CAR.MYVI_PSD: 0.35, CAR.ALZA: 0.4})

NOT_CAN_CONTROLLED = set([CAR.ARUZ, CAR.AXIA, CAR.BEZZA, CAR.MYVI])

ACC_CAR = set([CAR.ALZA, CAR.ATIVA, CAR.MYVI_PSD])
