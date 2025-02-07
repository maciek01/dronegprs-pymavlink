#!/usr/bin/env python


import modem,time




print(modem.strip00("00110022003300"))
print(modem.strip00("1100220033"))

print(modem.hex2ascii("53746174"))
print(modem.hex2ascii("53746174") == "Stat")

