#!/usr/bin/env python


import modem,time


firstModem = modem.findModem([
	"/dev/ttyUSB0",
	"/dev/ttyUSB1",
	"/dev/ttyUSB2",
	"/dev/ttyUSB3",
	"/dev/ttyUSB4",
	"/dev/ttyUSB5",
	"/dev/ttyUSB6"], 38400)

print(firstModem)

