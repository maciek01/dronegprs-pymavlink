#!/usr/bin/env python


import gps,time

gps.gpsinit("/dev/serial0", 38400)
#gps.gpsinit("/dev/serial0", 300)
#gps.gpsinit("/dev/serial0", 600)
#gps.gpsinit("/dev/serial0", 1200)
#gps.gpsinit("/dev/serial0", 2400)
#gps.gpsinit("/dev/serial0", 4800)
#gps.gpsinit("/dev/serial0", 9600)
#gps.gpsinit("/dev/serial0", 14400)
#gps.gpsinit("/dev/serial0", 19200)
#gps.gpsinit("/dev/serial0", 57600)
#gps.gpsinit("/dev/serial0", 115200)
#gps.gpsinit("/dev/serial0", 128000)
#gps.gpsinit("/dev/serial0", 256000)

while True:
	print "Lat/Lon: " + gps.GPSLAT + " / " + gps.GPSLON
	print "Lat: " + str(gps.GPSLATNORM)
	print "Lon: " + str(gps.GPSLONNORM)
	print "Alt: " + str(gps.GPSALT)
	print "Speed: " + str(gps.GPSSPEED)
	print "Heading: " + str(gps.GPSHEADING)
	print "Time UTC: " + gps.GPSTIME
	print "Status: " + gps.GPSSTATUS
	print "Status ts: ", gps.GPSLASTSTATUSMS
	print ""
	time.sleep(1)


