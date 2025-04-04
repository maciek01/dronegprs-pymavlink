#!/usr/bin/env python3


import sys, traceback
import threading
import time, datetime, json
import asyncio
from pymavlink import mavutil
import numpy
import math
import threading


current_milli_time = lambda: int(time.time() * 1000)

operatingAlt = 100
operatingSpeed = 15
requestedLat = None
requestedLon = None
savedLat = None
savedLon = None


URL = None
BAUD = None
TYPE = None
log = None
cmdLRReversed = False
cmdBFReversed = False

#monitoring and controlling tasks

vehicleLock = threading.RLock()

task = None
telem_thread = None
telem_on = False

control_thread = None
control_on = False

pilot_on = False
pilot_thread = None

#MAVLINK status


#mavlink connection
the_connection = None
#messages
gpsRaw = None
gps2Raw = None
globalPos = None
home = None
battery = None
rangefinder = None
statustext = None
sysstatus = None
heartbeat = None
#calculated
armed = None
armedStatus	= None
flightMode = None
#mavPosition = None
fixType = None
fix2Type = None


#controls
centerTheThrottle = False

ch1Override = 0
ch2Override = 0
ch3Override = 0
ch4Override = 0

#consants

fix_type_text = {
	mavutil.mavlink.GPS_FIX_TYPE_NO_GPS: "No GPS",
	mavutil.mavlink.GPS_FIX_TYPE_NO_FIX: "No Fix",
	mavutil.mavlink.GPS_FIX_TYPE_2D_FIX: "2D Fix",
	mavutil.mavlink.GPS_FIX_TYPE_3D_FIX: "3D Fix",
	mavutil.mavlink.GPS_FIX_TYPE_DGPS: "DGPS",
	mavutil.mavlink.GPS_FIX_TYPE_RTK_FLOAT: "RTK Float",
	mavutil.mavlink.GPS_FIX_TYPE_RTK_FIXED: "RTK Fixed",
	mavutil.mavlink.GPS_FIX_TYPE_STATIC: "Static",
	mavutil.mavlink.GPS_FIX_TYPE_PPP: "PPP"
}

########################### STATE AND MESSAGE OBSERVERS ########################


def setMessageFrequency(message_id, frequency):
	global the_connection
	message = the_connection.mav.command_long_encode(
		the_connection.target_system,  # Target system ID
		the_connection.target_component,  # Target component ID
		mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
		0,  # Confirmation
		message_id,  # param1: Message ID to be streamed
		frequency, # param2: Interval in microseconds
		0,  # param3 (unused)
		0,  # param4 (unused)
		0,  # param5 (unused)
		0,  # param5 (unused)
		0   # param6 (unused)
	)

	the_connection.mav.send(message)

	# Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
	response = the_connection.recv_match(type='COMMAND_ACK', blocking=False)
	if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
		log.info("Command accepted for message ID %d with frequency %d microseconds" % (message_id, frequency))
		return True
	else:
		log.info("Command failed for message ID %d" % message_id)
		return False

def controlMonitor():

	global centerTheThrottle
	global control_on

	centerTheThrottle = False
	while control_on:
		time.sleep(1)
		if centerTheThrottle:
			centerThrottle()



def telemMonitor():

	global telem_on

	global the_connection
	global gpsRaw
	global gps2Raw
	global globalPos
	global home
	global battery
	global rangefinder
	global statustext
	global sysstatus
	global heartbeat
	global flightMode
	global armed
	global armedStatus
	#global mavPosition
	global fixType
	global fix2Type
	global fix_type_text


	log.info("CONNECTING TELEMETRY")
	while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 5000000):
		time.sleep(1)
	while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1000000):
		time.sleep(1)
	while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_GPS2_RAW, 1000000):
		time.sleep(1)
	while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000):
		time.sleep(1)
	while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 5000000):
		time.sleep(1)
	while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 1000000):
		time.sleep(1)
	#while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 1000000):
	#	time.sleep(1)
	while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000):
		time.sleep(1)
	while not setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1000000):
		time.sleep(1)
	log.info("TELEMETRY CONNECTED")

	while telem_on:
		try:
			time.sleep(1)
			gpsRaw = the_connection.recv_match(type='GPS_RAW_INT', blocking=False)
			gps2Raw = the_connection.recv_match(type='GPS_RAW_INT', blocking=False)
			globalPos = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
			battery = the_connection.recv_match(type='BATTERY_STATUS', blocking=False)
			home = the_connection.recv_match(type='HOME_POSITION', blocking=False)
			rangefinder = the_connection.recv_match(type='RANGEFINDER', blocking=False)
			statustext = the_connection.recv_match(type='STATUSTEXT', blocking=False)
			sysstatus = the_connection.recv_match(type='SYS_STATUS', blocking=False)
			heartbeat = the_connection.recv_match(type='HEARTBEAT', blocking=False)

		except Exception as e:
			log.info("error in monitor receive", e)

		try:

			if heartbeat is None:
				heartbeat = getMessage('HEARTBEAT')
			if sysstatus is None:
				sysstatus = getMessage('SYS_STATUS')
			if statustext is None:
				statustext = getMessage('STATUSTEXT')
			if rangefinder is None:
				rangefinder = getMessage('RANGEFINDER')
			if home is None:
				home = getMessage('HOME_POSITION')
			if battery is None:
				battery = getMessage('BATTERY_STATUS')
			if globalPos is None:
				globalPos = getMessage('GLOBAL_POSITION_INT')
			if gpsRaw is None:
				gpsRaw = getMessage('GPS_RAW_INT')
			if gps2Raw is None:
				gps2Raw = getMessage('GPS2_RAW')


			#calculated telem state

			flightMode = the_connection.flightmode
			armed = bool(the_connection.motors_armed())
			if armed:
				armedStatus = "ARMED"
			else:
				armedStatus = "DISARMED"
			#works with sim, blocks in real life
			#mavPosition = the_connection.location()
			fixType = fix_type_text.get(
				gpsRaw.fix_type if gpsRaw != None else -1,
				"Unknown"
				)
			fix2Type = fix_type_text.get(
				gps2Raw.fix_type if gps2Raw != None else -1,
				"Unknown"
				)

		except Exception as e:
			log.info("error in monitor get", e)


def getMessage(msg):
	global the_connection

	try:
		return the_connection.messages[msg]
	except:
		return None


########################### THREAD HELPERS #####################################

def lockV():
	global vehicleLock
	vehicleLock.acquire()

def unlockV():
	global vehicleLock
	vehicleLock.release()

def initVehicle():
	global URL
	global BAUD
	global the_connection
	global log
	global telem_thread
	global telem_on
	global control_thread
	global control_on


	lockV()
	try:
		if telem_thread != None:
			try:
				telem_on = False
				telem_thread.join()
				telem_thread = None
				log.info("TELEM THREAD JOINED")
			except:
				log.info("ERROR WHEN CANCELLING A TELEM TASK")

		if control_thread != None:
			try:
				control_on = False
				control_thread.join()
				control_thread = None
				log.info("CONTROL THREAD JOINED")
			except:
				log.info("ERROR WHEN CANCELLING A CONTROL TASK")

		#close conn
		try:
			the_connection = None
		except Exception as inst:
			the_connection = None

		#open conn
		while the_connection == None:
			try:
				# Init the drone

				# Start a connection listening to a UDP port
				the_connection = mavutil.mavlink_connection(URL, BAUD)

				# Wait for the first heartbeat
				#   This sets the system and component ID of remote system for the link
				the_connection.wait_heartbeat()
				log.info("Heartbeat from system (system %u component %u)" %
					(the_connection.target_system, the_connection.target_component))


			except Exception as inst:
				the_connection = None
				traceback.print_exc()
				time.sleep(5)


		#register listeners

		telem_on = True
		telem_thread = threading.Thread(target=telemMonitor, args=())
		telem_thread.daemon = True
		telem_thread.start()


		control_on = True
		control_thread = threading.Thread(target=controlMonitor, args=())
		control_thread.daemon = True
		control_thread.start()


	finally:
		unlockV()



####################### MAIN THREAD ############################################

def pilotMonitor():

	global the_connection
	global telem_thread
	global telem_on
	global control_thread
	global control_on
	global log
	global pilot_on

	#wait to initialize the pilot
	log.info("ABOUT to call INIT VEH")

	initVehicle()



#	telem_on = True
#	telem_thread = threading.Thread(target=telemMonitor, args=())
#	telem_thread.daemon = True
#	telem_thread.start()


#	control_on = True
#	control_thread = threading.Thread(target=controlMonitor, args=())
#	control_thread.daemon = True
#	control_thread.start()


	#read loop

	while pilot_on:
		try:
			time.sleep(1)
			#check if the_connection is still alive


			#todo: check if the connection is still alive - look for heartbeat last time
			#if old - reinit: break the thread loops and reinit
			if the_connection.time_since('HEARTBEAT') > 30:
				log.info("!!! HEARTBEAT OLD - REINIT VEH")
				initVehicle()

		except Exception as inst:
			#traceback.print_exc()
			initVehicle()



###################### INIT HANDLER ############################################
	
async def pilotinit(_log, url, baud, type, _cmdLRReversed, _cmdBFReversed):
	global URL
	global BAUD
	global TYPE
	global telem_thread
	global telem_on
	global log
	global task
	global pilot_on
	global pilot_thread
	global cmdLRReversed
	global cmdBFReversed

	log = _log
	URL = url
	TYPE = type
	BAUD = baud
	cmdLRReversed = _cmdLRReversed
	cmdBFReversed = _cmdBFReversed

	pilot_on = True
	pilot_thread = threading.Thread(target=pilotMonitor, args=())
	pilot_thread.daemon = True
	pilot_thread.start()


	#task = asyncio.create_task(coro=pilotMonitor(), name="pilot")
	#await asyncio.sleep(1)


############################ LOACAL FUNCs     ##################################

def get_bearing(lat1, long1, lat2, long2):
	dLon = (long2 - long1)
	x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
	y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
	brng = numpy.arctan2(x,y)

	return brng

def get_bearingDeg(lat1, long1, lat2, long2):
	brng = numpy.degrees(get_bearing(lat1, long1, lat2, long2))

	return brng

############################ COMMAND HANDLERS ##################################

async def arm(data):

	global the_connection
	global log
	global savedLat
	global savedLon

	lockV()

	try:
		log.info("ARM")

		try:

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
				0, 1, 0, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"Arm ACK:  {msg}")

		except:
			traceback.print_exc()

		releaseThrottle()
		
		#cancel resume
		savedLat = None
		savedLon = None		
		
		return "OK"	
		
	finally:
		unlockV()

async def disarm(data):

	global the_connection
	global log
	global savedLat
	global savedLon

	lockV()
	try:
		log.info("DISARM")
			
		try:

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
				0, 0, 0, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"Disarm ACK:  {msg}")

		except:
			traceback.print_exc()

		releaseThrottle()

		#cancel resume
		savedLat = None
		savedLon = None		
	
		log.info(" disarming")
		return "OK"	
		
	finally:
		unlockV()


#helper for guided commands
async def guided():

	global the_connection
	global log

	lockV()
	try:
		log.info("GUIDED")
			
		try:

			mode_id = the_connection.mode_mapping()["GUIDED"]

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_SET_MODE,
				0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"GUIDED ACK:  {msg}")

		except:
			traceback.print_exc()
			
		log.info(" guided")
		return "OK"	
		
	finally:
		unlockV()




async def takeoff(data):

	global the_connection
	global log
	global TYPE
	global operatingAlt
	global armed
	global savedLat
	global savedLon

	lockV()
	try:
	
		aTargetAltitude = operatingAlt

		takeoff_params = [0, 0, 0, 0, 0, 0, float(aTargetAltitude)]

		try:
			await arm(data)
			await guided()
		except:
			traceback.print_exc()

		log.info("TAKEOFF")

		i = 0
		while not armed and i < 10:
			i = i + 1
			await asyncio.sleep(1)

		if not armed:
			log.info(" NOT ARMED")
			return "ERROR: NOT ARMED IN 10 secs"
			
		try:
			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF if TYPE == "vtol" else mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
				0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3],
				takeoff_params[4], takeoff_params[5], takeoff_params[6])
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"TAKEOFF ACK:  {msg}")
		except:
			traceback.print_exc()

		releaseThrottle()

		#cancel resume
		savedLat = None
		savedLon = None		
		
		log.info(" took off")
			

		return "OK"
	finally:
		unlockV()

async def land(data):

	global the_connection
	global requestedLat
	global requestedLon
	global savedLat
	global savedLon
	global log
	global TYPE
	
	lockV()
	try:
		log.info("LAND")

		try:
			await guided()
		except:
			traceback.print_exc()
			
		try:
			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_NAV_VTOL_LAND if TYPE == "vtol" else mavutil.mavlink.MAV_CMD_NAV_LAND, 
				0, 0, 0, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"LAND ACK:  {msg}")

		except:
			traceback.print_exc()

		releaseThrottle()

		#cancel last goto		
		savedLat = None
		savedLon = None		
		requestedLat = None
		requestedLon = None
		
		log.info(" landing")
	
		return "OK"

	finally:
		unlockV()


# helper for mode based commands
async def modeCmd(mode):

	global the_connection
	global log

	try:

		mode_id = the_connection.mode_mapping()[mode]

		the_connection.mav.command_long_send(
			the_connection.target_system, the_connection.target_component,
			mavutil.mavlink.MAV_CMD_DO_SET_MODE,
			0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
		
		msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
		log.info(f"{mode} ACK:  {msg}")

	except:
		traceback.print_exc()

#helper for hold modes
async def holdInMode(mode, modeAlias, overrideSticks = True):

	global requestedLat
	global requestedLon
	global savedLat
	global savedLon 
	global log
	
	lockV()
	try:
		log.info(modeAlias)
			
		if overrideSticks:
			centerThrottle()

		await modeCmd(mode)
		
		#save last goto
		savedLat = requestedLat
		savedLon = requestedLon
		requestedLat = None
		requestedLon = None

		if not overrideSticks:
			releaseThrottle()
		
		log.info(modeAlias + " activated")
			
		return "OK"

	finally:
		unlockV()		


async def position(data):
	return await holdInMode("POSHOLD", "POSHOLD")

async def loiter(data):
	global TYPE
	return await holdInMode("QLOITER" if TYPE == "vtol" else "LOITER", "LOITER")

async def pause(data):
	global TYPE
	return await holdInMode("QLOITER" if TYPE == "vtol" else "LOITER", "PAUSE")

async def manual(data):
	global TYPE
	return await holdInMode("QLOITER" if TYPE == "vtol" else "LOITER", "MANUAL", False)


async def rtl(data):

	global requestedLat
	global requestedLon
	global operatingSpeed
	global savedLat
	global savedLon	   
	global log
	
	lockV()
	try:
		log.info("RTL")

		try:
			await modeCmd("RTL")
		except:
			traceback.print_exc()
		try:
			res = sendSpeedMsg(float(operatingSpeed))
		except:
			traceback.print_exc()

		releaseThrottle()

		#cancel last goto
		savedLat = None
		savedLon = None		
		requestedLat = None
		requestedLon = None
		
		log.info(" returning home")

		return "OK"

	finally:
		unlockV()
		
async def auto(data):

	global requestedLat
	global requestedLon
	global savedLat
	global savedLon 
	global log
	
	lockV()
	try:
		log.info("AUTO")
			
		try:
			await modeCmd("AUTO")
		except:
			traceback.print_exc()
		
		releaseThrottle()

		#save last goto
		savedLat = None
		savedLon = None
		requestedLat = None
		requestedLon = None
		
		
		log.info(" on auto mission")

		return "OK"

	finally:
		unlockV()
		

#helper for set position
async def reposition(adjHeading = True):

	global the_connection
	global operatingAlt
	global requestedLat
	global requestedLon
	global operatingSpeed
	global log
	global globalPos
	
	if adjHeading:
		brg = get_bearing(globalPos.lat / 10 ** 7, globalPos.lon / 10 ** 7,
			float(requestedLat), float(requestedLon))
	else:
		brg = -1

	try:
		await guided()
	except:
		traceback.print_exc()

	try:

		the_connection.mav.send(
			mavutil.mavlink.MAVLink_set_position_target_global_int_message(
				10, #time_boot_ms
				the_connection.target_system,
				the_connection.target_component,
				mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
				int(0b110111111000),
				int(float(requestedLat) * 10 ** 7),
				int(float(requestedLon) * 10 ** 7),
				float(operatingAlt), 0, 0, 0, 0, 0, 0,
				brg,
				0.5))

		msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
		log.info(f"SET POSITION ACK:  {msg}")

	except:
		traceback.print_exc()

	try:
		res = sendSpeedMsg(float(operatingSpeed))
	except:
		traceback.print_exc()

	releaseThrottle()

async def goto(data):

	global requestedLat
	global requestedLon
	global log
	global savedLat
	global savedLon
	
	lockV()
	try:
		log.info("GOTO")
		if not armed:
			log.info(" NOT ARMED")
			return "ERROR: NOT ARMED"

		parameters = data['command']['parameters']
		
		for i in parameters:
			if i['name'] == "lat":
				lat = i['value']
				requestedLat = lat
			if i['name'] == "lon":
				lon = i['value']
				requestedLon = lon

		await reposition()

		#cancel resume
		savedLat = None
		savedLon = None

		log.info(" going to ")

		return "OK"

	finally:
		unlockV()

async def resume(data):

	global the_connection
	global globalPos
	global home
	global requestedLat
	global requestedLon
	global operatingSpeed
	global savedLat
	global savedLon 
	global log
	global centerTheThrottle
	
	lockV()
	try:
		log.info("RESUME")

		if savedLat != None and savedLon != None:
			requestedLat = savedLat
			requestedLon = savedLon

			await reposition()

			#cancel resume
			savedLat = None
			savedLon = None
			
			log.info(" resuming")
		else:
			log.info(" not resuming - no saved position")
			
		return "OK"

	finally:
		unlockV()

async def reHome(data):
	global the_connection
	global log
	
	lockV()
	try:
		log.info("REHOME")

		the_connection.mav.command_long_send(
			the_connection.target_system, the_connection.target_component,
			mavutil.mavlink.MAV_CMD_DO_SET_HOME,0,1,0,0,0,0,0,0)

		msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
		log.info(f"SETHOME ACK:  {msg}")

		log.info(" rehoming")

		return "OK"

	finally:
		unlockV()

async def setHome(data):
	global the_connection
	global log
	global home
	lat = None
	lon = None
	lockV()
	try:
		log.info("SETHOME")
		
		parameters = data['command']['parameters']
		
		for i in parameters:
			if i['name'] == "lat":
				lat = i['value']
			if i['name'] == "lon":
				lon = i['value']
		
		if lat == None or lon == None:
			log.info("NO COORDS")
			return "ERROR: NO COORDS"
		
		the_connection.mav.command_long_send(
			the_connection.target_system, the_connection.target_component,
			mavutil.mavlink.MAV_CMD_DO_SET_HOME,0,0,0,0,0,
				float(lat),
				float(lon),
				home.altitude / 1000)

		msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
		log.info(f"SETHOME ACK:  {msg}")

		log.info(" rehoming")

		return "OK"

	finally:
		unlockV()

		
async def alt(data):

	global operatingAlt
	global log
	
	lockV()
	try:
		log.info("ALT")
		
		parameters = data['command']['parameters']
		
		for i in parameters:
			if i['name'] == "alt":
				operatingAlt = i['value']

				changeAlt(operatingAlt)
		
		log.info("new operating alt is now " + operatingAlt)
		return "OK"

	finally:
		unlockV()
		
async def altAdjust(delta):

	global operatingAlt
	global log
	
	lockV()
	try:
		operatingAlt = str(max(int(operatingAlt) + delta, 0))
		
		await changeAlt(operatingAlt)

		log.info("adjusted operating alt is now " + operatingAlt)

		return "OK"

	finally:
		unlockV()		

async def setToCurrAlt(data):
	global operatingAlt
	global globalPos
	global log


	lockV()
	try:
		log.info("SETCURRALT")

		operatingAlt = str(max(int(globalPos.relative_alt / 1000), 0))

		await changeAlt(operatingAlt)

		log.info("adjusted operating alt is now " + operatingAlt)

		return "OK"

	finally:
		unlockV()

async def changeAlt(relAlt):

	global requestedLat
	global requestedLon
	global globalPos

	if requestedLat == None or requestedLon == None:
		try:
			requestedLat = globalPos.lat / 10 ** 7
			requestedLon = globalPos.lon / 10 ** 7
			await reposition(adjHeading = False)
		except:
			traceback.print_exc()
		finally:
			requestedLat = None
			requestedLon = None
	else:
		try:
			await reposition()
		except:
			traceback.print_exc()

#not supported by AP yet
async def __changeAlt(relAlt):

	global the_connection
	global operatingAlt

	#wont work in LOITER mode
	try:


		#not supported by AP yet
		the_connection.mav.command_long_send(
			the_connection.target_system, the_connection.target_component,
			mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
			0,float(operatingAlt),
			mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,0,0,0,0,0,)

		msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
		log.info(f"ALT ACK:  {msg}")

		#0 - SUCESS/ACCEPTED
		#1 - TEMPORARY REJECTED
		#2-  DENIED
		#3 - COMMAND UNSUPPORTED
		#4 - FAILED
		#5 - IN PROGRESS
		#more see https://mavlink.io/en/messages/common.html#MAV_RESULT

		return msg.result if msg != None else -1 #-1 is timeout

	except:
		traceback.print_exc()

	return -1

def sendSpeedMsg(spd):
	global the_connection
	global log

	log.info("sending :  MAV_CMD_DO_CHANGE_SPEED: " + str(spd))

	the_connection.mav.command_long_send(
	the_connection.target_system, the_connection.target_component,
	mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1,spd,0,0,0,0,0,)

	msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
	log.info(f"SPD ACK:  {msg}")

	#0 - SUCESS/ACCEPTED
	#1 - TEMPORARY REJECTED
	#2-  DENIED
	#3 - COMMAND UNSUPPORTED
	#4 - FAILED
	#5 - IN PROGRESS
	#more see https://mavlink.io/en/messages/common.html#MAV_RESULT

	return msg.result if msg != None else -1 #-1 is timeout

async def speed(data):

	global the_connection
	global operatingSpeed
	global log
	
	lockV()
	try:
		log.info("SPEED")
		
		parameters = data['command']['parameters']
		
		for i in parameters:
			if i['name'] == "speed":
				spd = i['value']

				res = sendSpeedMsg(float(spd))
				if res == 0:
					operatingSpeed = spd
					log.info(" operating speed is now " + operatingSpeed)
					return "OK"
		return "FAILED"
	finally:
		unlockV()
		
async def speedAdjust(delta):
	global the_connection
	global operatingSpeed
	global log
	
	lockV()
	try:
		spd = str(max(min(int(operatingSpeed) + delta, 20), 1))

		res = sendSpeedMsg(float(spd))
		if res == 0:
			operatingSpeed = spd
			log.info(" operating speed is now " + operatingSpeed)
			return "OK"

		return "FAILED"

	finally:
		unlockV()

async def decAlt1(data):
	global log
	log.info("DECALT1")
	return await altAdjust(-1)
	
async def decAlt10(data):
	global log
	log.info("DECALT10")
	return await altAdjust(-10)
	
async def incAlt10(data):
	global log
	log.info("INCALT10")
	return await altAdjust(10)
	
async def incAlt1(data):
	global log
	log.info("INCALT1")
	return await altAdjust(1)
	
async def decSpeed1(data):
	global log
	log.info("DECSPEED1")
	return await speedAdjust(-1)
	
async def decSpeed10(data):
	global log
	log.info("DECSPEED10")
	return await speedAdjust(-10)
	
async def incSpeed10(data):
	global log
	log.info("INCSPEED10")
	return await speedAdjust(10)
	
async def incSpeed1(data):
	global log
	log.info("INCSPEED1")
	return await speedAdjust(1)

async def moveLeft(data):
	global log
	global ch1Override
	global ch2Override
	global ch3Override
	global ch4Override
	global cmdLRReversed
	
	log.info("MVLEFT")
	
	if cmdLRReversed:
		ch1Override = 1700
	else:
		ch1Override = 1300

	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)
	await asyncio.sleep(0.5)
	ch1Override = 0
	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)

	return "OK"

async def moveRight(data):
	global log
	global ch1Override
	global ch2Override
	global ch3Override
	global ch4Override
	global cmdLRReversed

	log.info("MVRIGHT")
	
	if cmdLRReversed:
		ch1Override = 1300
	else:
		ch1Override = 1700

	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)
	await asyncio.sleep(0.5)
	ch1Override = 0
	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)

	return "OK"

async def moveForward(data):
	global log
	global ch1Override
	global ch2Override
	global ch3Override
	global ch4Override
	global cmdBFReversed

	log.info("MVFWD")

	if cmdBFReversed:
		ch2Override = 1700
	else:
		ch2Override = 1300

	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)
	await asyncio.sleep(0.5)
	ch2Override = 0
	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)

	return "OK"

async def moveBack(data):
	global log
	global ch1Override
	global ch2Override
	global ch3Override
	global ch4Override
	global cmdBFReversed

	log.info("MVBCK")

	if cmdBFReversed:
		ch2Override = 1300
	else:
		ch2Override = 1700
	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)
	await asyncio.sleep(0.5)
	ch2Override = 0
	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)

	return "OK"

def setChannells(ch1 = 0, ch2 = 0, ch3 = 0, ch4 = 0, ch5 = 0, ch6 = 0, ch7 = 0, ch8 = 0):
	global the_connection

	log.info("OVERRIDE: " + str(ch1) + " " + str(ch2) + " " + str(ch3) + " " + str(ch4)
		+ " " + str(ch5) + " " + str(ch6) + " " + str(ch7) + " " + str(ch8))

	the_connection.mav.send(
		mavutil.mavlink.MAVLink_rc_channels_override_message(
				the_connection.target_system, the_connection.target_component,
				ch1,
				ch2,
				ch3,
				ch4,
				ch5,
				ch6,
				ch7,
				ch8,))

#override throttle - center - this is to make sure the drone does not descend in non guided modes 
# make sure to set timeout to 120 seconds
# https://ardupilot.org/copter/docs/parameters.html#rc-override-time
def centerThrottle():
	global centerTheThrottle
	global ch1Override
	global ch2Override
	global ch3Override
	global ch4Override

	ch3Override = 1500
	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)
	centerTheThrottle = True
	
#remove throttle overrides
def releaseThrottle():
	global centerTheThrottle
	global ch1Override
	global ch2Override
	global ch3Override
	global ch4Override

	ch3Override = 0
	setChannells(ch1 = ch1Override, ch2 = ch2Override, ch3 = ch3Override, ch4 = ch4Override)
	centerTheThrottle = False
	

	


