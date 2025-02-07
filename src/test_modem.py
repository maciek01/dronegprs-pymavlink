#!/usr/bin/env python

import asyncio
import modem,time
import logger

modem.log = logger.setup_custom_logger('main')


async def run():

	print ("USB0:", await modem.isModem("/dev/ttyUSB0", 38400))
	print ("USB1:", await modem.isModem("/dev/ttyUSB1", 38400))
	print ("USB2:", await modem.isModem("/dev/ttyUSB2", 38400))
	print ("USB3:", await modem.isModem("/dev/ttyUSB3", 38400))
	print ("USB4:", await modem.isModem("/dev/ttyUSB4", 38400))
	print ("USB5:", await modem.isModem("/dev/ttyUSB5", 38400))
	print ("USB6:", await modem.isModem("/dev/ttyUSB6", 38400))

	firstModem = await modem.findModem([
		"/dev/ttyUSB0",
		"/dev/ttyUSB1",
		"/dev/ttyUSB2",
		"/dev/ttyUSB3"], 38400)

	print(firstModem)

	if firstModem != "":
		await modem.modeminit(firstModem, 38400, 5, True)
		while True:
			print("Status: |" + modem.MODEMSTATUS + "|")
			print("Signal: |" + modem.MODEMSIGNAL + "|")
			print("")
			time.sleep(1)


#main
if __name__ == '__main__':
        asyncio.run(run())

