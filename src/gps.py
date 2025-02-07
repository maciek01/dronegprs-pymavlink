#!/usr/bin/env python

import sys, traceback
import serial, threading
import time, datetime
import Main


readOn = True

buffer = ""
getLine = ""

thread = None

GPSLAT = ""
GPSLON = ""
GPSLATNORM = 0
GPSLONNORM = 0
GPSALT = 0
GPSTIME = ""
GPSSTATUS = "INVALID"
GPSTIMESTAMP = 0
GPSLASTSTATUSMS = 0
GPSSPEED = 0
GPSHEADING = 0
GPSNSATS = 0
GPSFIX = 0

serialPort = None

current_milli_time = lambda: int(time.time() * 1000)

def handle_newline(line):
	global GPSLAT
	global GPSLATNORM
	global GPSLON
	global GPSLONNORM
	global GPSALT
	global GPSTIME
	global GPSSTATUS
	global GPSLASTSTATUSMS
	global GPSSPEED
	global GPSHEADING
	global GPSNSATS
	global GPSFIX
	

	if line.startswith("$GPGLL") or line.startswith("$GNGLL") or line.startswith("$GLGLL"):
		#print line
		elements = line.split(',')
		lat = elements[1]
		ns = elements[2]
		lon = elements[3]
		ew = elements[4]
		gpstime = elements[5]
		status = elements[6]

		parseStdData(lat, ns, lon, ew, gpstime)

		if status == "A":
			GPSSTATUS = "VALID"
		else:
			GPSSTATUS = "INVALID"
		GPSLASTSTATUSMS = current_milli_time()

	if line.startswith("$GNGGA"):
		#print line
		elements = line.split(',')
		lat = elements[2]
		ns = elements[3]
		lon = elements[4]
		ew = elements[5]
		gpstime = elements[1]

		parseStdData(lat, ns, lon, ew, gpstime)

		GPSSTATUS = "UNKNOWN"
		GPSLASTSTATUSMS = current_milli_time()
		GPSALT=float(elements[9])
		GPSNSATS = int(elements[7])
		GPSFIX = int(elements[6])

	if line.startswith("$GNRMC") and False:
		#print line
		elements = line.split(',')
		lat = elements[3]
		ns = elements[4]
		lon = elements[5]
		ew = elements[6]
		gpstime = elements[1]
		status = elemenets[2]

		parseStdData(lat, ns, lon, ew, gpstime)

		if status == "A":
			GPSSTATUS = "VALID"
		else:
			GPSSTATUS = "INVALID"
		GPSLASTSTATUSMS = current_milli_time()

		speed = float(elements[7])
		heading = float(elements[8] if elements[8] != "" else 0)
		GPSSPEED = speed * 0.514444 #knots to mps
		GPSHEADNING = heading

	if line.startswith("$GNVTG"):
		#print line
		elements = line.split(',')
		speed = float(elements[7])
		heading = float(elements[1] if elements[1] != "" else 0)
		GPSSPEED = speed * 0.277778 #kmph to mps
		GPSHEADNING = heading

def parseStdData(lat, ns, lon, ew, gpstime):

	global GPSLAT
	global GPSLATNORM
	global GPSLON
	global GPSLONNORM
	global GPSTIME

	if lat != "":
		GPSLAT = lat[:2] + " " + lat[2:10] + " " + ns
		GPSLATNORM = (float(lat[:2]) + float(lat[2:10]) / 60.0) * (1 if ns.lower() == "n" else -1)
	if lon != "":
		GPSLON = lon[:3] + " " + lon[3:11] + " " + ew
		GPSLONNORM = (float(lon[:3]) + float(lon[3:11]) / 60.0) * (1 if ew.lower() == "e" else -1)
	if gpstime != "":
		GPSTIME = gpstime[:2] + ":" + gpstime[2:4] + ":" + gpstime[4:]

def handle_data(data):
	global buffer
	global getLine
	for d in data:
		if d == '\n':
			getLine = buffer
			buffer = ""
			#print (getLine)
			try:
				handle_newline(getLine)
			except Exception as inst:
				traceback.print_exc()
				Main.log.info("line:"+getLine)
				time.sleep(5)
		else:
			buffer = buffer + str(d)

def read_from_port(gpsport, gpsbaud):
	global readOn
	global serialPort

	#wait to initialize the port
	while serialPort == None:
		try:
			serialPort = serial.Serial(gpsport, baudrate=gpsbaud, timeout=None,
				parity = serial.PARITY_NONE,
				stopbits = serial.STOPBITS_ONE,
				bytesize = serial.EIGHTBITS)
		except Exception as inst:
			serialPort = None
			traceback.print_exc()
			time.sleep(5)

	#read loop
	ser = serialPort
	while readOn:
		try:
			if ser.inWaiting() > 0:
				#handle_data(ser.read(ser.inWaiting()))
				newl = ser.readline()
				#b = bytearray()
				#b.extend(newl)
				#print map(hex, b)
				#print newl
				handle_newline(newl)


		except Exception as inst:
			traceback.print_exc()
	
def gpsinit(gpsport, gpsbaud):
	global thread
	thread = threading.Thread(target=read_from_port, args=(gpsport,gpsbaud,))
	thread.daemon = True
	thread.start()



