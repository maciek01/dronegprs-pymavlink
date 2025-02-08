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
log = None

#monitoring tasks

vehicleLock = threading.RLock()

task = None
monitor_thread = None
monitor_on = False

#MAVLINK status
'''curr_tot = 0
voltages = None
bat_percent = 0
heading = 0
statusMessage = ""
statusSev = -1 #https://mavlink.io/en/messages/common.html#MAV_SEVERITY
gps_info = None
pos = None
gps_data = None
home = None
mode = None
armed = None
connected = False'''


#mavlink connection
the_connection = None
#messages
gpsRaw = None
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
mavPosition = None
fixType = None



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
	response = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
	if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
		log.info("Command accepted for message ID %d with frequency %d microseconds" % (message_id, frequency))
	else:
		log.info("Command failed for message ID %d" % message_id)



def telemMonitor():

	global monitor_on

	global the_connection
	global gpsRaw
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
	global mavPosition
	global fixType



	setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 5000000)
	setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1000000)
	setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000)
	setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 5000000)
	setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 1000000)
	setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 1000000)
	setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000)
	setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1000000)


	while monitor_on:
		try:
			time.sleep(1)
			#print("Waiting for telem")
			gpsRaw = the_connection.recv_match(type='GPS_RAW_INT', blocking=False)
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


			flightMode = the_connection.flightmode
			armed = bool(the_connection.motors_armed())
			if armed:
				armedStatus = "ARMED"
			else:
				armedStatus = "DISARMED"
			mavPosition = the_connection.location()

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
	vehicleLock.acquire()

def unlockV():
	vehicleLock.release()

async def initVehicle():
	global URL
	global BAUD
	global the_connection
	global log
	global monitor_thread
	global monitor_on


	lockV()
	try:
		if monitor_thread != None:
			try:
				monitor_on = False
				monitor_thread.join()
				monitor_thread = None
			except:
				log.info("ERROR WHEN CANCELLING A TASK")

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
				the_connection = mavutil.mavlink_connection(URL)

				# Wait for the first heartbeat
				#   This sets the system and component ID of remote system for the link
				the_connection.wait_heartbeat()
				log.info("Heartbeat from system (system %u component %u)" %
					(the_connection.target_system, the_connection.target_component))



			except Exception as inst:
				the_connection = None
				traceback.print_exc()
				await asyncio.sleep(5)


		#register listeners


	finally:
		unlockV()



####################### MAIN THREAD ############################################

async def pilotMonitor():

	global the_connection
	global monitor_thread
	global monitor_on
	global log

	#wait to initialize the pilot
	log.info("ABOUT to call INIT VEH")

	await initVehicle()

	monitor_on = True
	monitor_thread = threading.Thread(target=telemMonitor, args=())
	monitor_thread.daemon = True
	monitor_thread.start()

	#read loop

	while True:
		try:
			await asyncio.sleep(1)
			#check if the_connection is still alive
		except Exception as inst:
			#traceback.print_exc()
			await initVehicle()



###################### INIT HANDLER ############################################
	
async def pilotinit(_log, url, baud):
	global URL
	global BAUD
	global monitor_thread
	global monitor_on
	global log
	global task

	log = _log

	URL = url
	BAUD = baud

	task = asyncio.create_task(coro=pilotMonitor(), name="pilot")
	await asyncio.sleep(1)


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

		await releaseSticks()
		
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
		await releaseSticks()
		
		#cancel resume
		savedLat = None
		savedLon = None		
	
		log.info(" disarming")
		return "OK"	
		
	finally:
		unlockV()



async def guided(data):

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
			
		log.info(" is guided")
		return "OK"	
		
	finally:
		unlockV()




async def takeoff(data):

	global the_connection
	global log
	global operatingAlt
	global armed
	global savedLat
	global savedLon

	lockV()
	try:
	
		aTargetAltitude = operatingAlt

		takeoff_params = [0, 0, 0, 0, 0, 0, aTargetAltitude]

		try:
			await arm(data)
			await guided(data)
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
				mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
				0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3],
				takeoff_params[4], takeoff_params[5], takeoff_params[6])
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"TAKEOFF ACK:  {msg}")
		except:
			traceback.print_exc()
		await releaseSticks()

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
	
	lockV()
	try:
		log.info("LAND")
		#if not armed:
			#print " NOT ARMED"
			#return "ERROR: NOT ARMED"
			
		try:
			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_NAV_LAND, 
				0, 0, 0, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"LAND ACK:  {msg}")

		except:
			traceback.print_exc()

		await releaseSticks()

		#cancel last goto		
		savedLat = None
		savedLon = None		
		requestedLat = None
		requestedLon = None
		
		log.info(" landing")
	
		return "OK"

	finally:
		unlockV()

async def position(data):

	global the_connection
	global requestedLat
	global requestedLon
	global savedLat
	global savedLon 
	global log
	
	lockV()
	try:
		log.info("POSITION")
			
		await centerSticks()
		try:

			mode_id = the_connection.mode_mapping()["POSHOLD"]

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_SET_MODE,
				0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"HOLD ACK:  {msg}")

		except:
			traceback.print_exc()
		
		#save last goto
		savedLat = requestedLat
		savedLon = requestedLon
		requestedLat = None
		requestedLon = None
		

		log.info(" holding position")
			
		return "OK"

	finally:
		unlockV()
		

async def loiter(data):

	global the_connection
	global requestedLat
	global requestedLon
	global savedLat
	global savedLon 
	global log
	
	lockV()
	try:
		log.info("LOITER")
			
		await centerSticks()
		try:

			mode_id = the_connection.mode_mapping()["LOITER"]

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_SET_MODE,
				0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"LOITER ACK:  {msg}")

		except:
			traceback.print_exc()
		
		#save last goto
		savedLat = requestedLat
		savedLon = requestedLon
		requestedLat = None
		requestedLon = None
		
		log.info(" loitering")
			
		return "OK"

	finally:
		unlockV()		

async def auto(data):

	global the_connection
	global requestedLat
	global requestedLon
	global savedLat
	global savedLon 
	global log
	
	lockV()
	try:
		log.info("AUTO")
			
		await releaseSticks()
		try:
			#await vehicle.mission.start_mission()

			mode_id = the_connection.mode_mapping()["AUTO"]

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_SET_MODE,
				0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"AUTO ACK:  {msg}")

		except:
			traceback.print_exc()
		
		#save last goto
		savedLat = None
		savedLon = None
		requestedLat = None
		requestedLon = None
		
		
		log.info(" on auto mission")

		return "OK"

	finally:
		unlockV()
		
async def pause(data):

	global the_connection
	global log
	
	lockV()
	try:
		log.info("PAUSE")
			
		
		if True:
			#just loiter
			try:

				mode_id = the_connection.mode_mapping()["LOITER"]

				the_connection.mav.command_long_send(
					the_connection.target_system, the_connection.target_component,
					mavutil.mavlink.MAV_CMD_DO_SET_MODE,
					0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
				
				msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
				log.info(f"LOITER ACK:  {msg}")

			except:
				traceback.print_exc()
			

		log.info(" paused")

		return "OK"

	finally:
		unlockV()

async def resume(data):

	global the_connection
	global mavPosition
	global home
	global requestedLat
	global requestedLon
	global operatingSpeed
	global savedLat
	global savedLon 
	global log
	
	lockV()
	try:
		log.info("RESUME")

		if savedLat != None and savedLon != None:
			requestedLat = savedLat
			requestedLon = savedLon

			await guided(data)

			brg = get_bearing(mavPosition.lat, mavPosition.lng, float(requestedLat), float(requestedLon))


			try:

				the_connection.mav.send(
					mavutil.mavlink.MAVLink_set_position_target_global_int_message(
						10, #time_boot_ms
						the_connection.target_system,
						the_connection.target_component,
						mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
						int(0b110111111000),
						int(-float(requestedLat) * 10 ** 7),
						int(float(requestedLon) * 10 ** 7),
						float(operatingAlt), 0, 0, 0, 0, 0, 0, brg, 0.5))

				msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
				log.info(f"GOTO ACK:  {msg}")

			except:
				traceback.print_exc()

			try:
				the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1,float(operatingSpeed),0,0,0,0,0,)

				msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
				log.info(f"SPD ACK:  {msg}")

			except:
				traceback.print_exc()


			savedLat = None
			savedLon = None
			await releaseSticks()
		
		log.info(" resuming")
			
		return "OK"

	finally:
		unlockV()

#release channel overrides
async def manual(data):

	global the_connection
	global log
	
	lockV()
	try:
		log.info("MANUAL")
		try:

			mode_id = the_connection.mode_mapping()["LOITER"]

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_SET_MODE,
				0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"LOITER ACK:  {msg}")

		except:
			traceback.print_exc()

		await releaseSticks()

		log.info(" manual control")

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
			mavutil.mavlink.MAV_CMD_DO_SET_HOME,0,1,0,0,0,0,0,0,)

		msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
		log.info(f"SETHOME ACK:  {msg}")

		log.info(" rehoming")

		return "OK"

	finally:
		unlockV()
		
async def rtl(data):

	global the_connection
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

			mode_id = the_connection.mode_mapping()["RTL"]

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_SET_MODE,
				0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
			
			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"RTL ACK:  {msg}")

		except:
			traceback.print_exc()

		try:
			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1,float(operatingSpeed),0,0,0,0,0,)

			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"SPD ACK:  {msg}")

		except:
			traceback.print_exc()

		await releaseSticks()

		#cancel last goto
		savedLat = None
		savedLon = None		
		requestedLat = None
		requestedLon = None
		
		log.info(" returning home")


		return "OK"

	finally:
		unlockV()
		
		
async def goto(data):

	global the_connection
	global home
	global operatingAlt
	global requestedLat
	global requestedLon
	global operatingSpeed
	global log
	
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


		brg = get_bearing(mavPosition.lat, mavPosition.lng, float(requestedLat), float(requestedLon))

		try:

			the_connection.mav.send(
				mavutil.mavlink.MAVLink_set_position_target_global_int_message(
					10, #time_boot_ms
					the_connection.target_system,
					the_connection.target_component,
					mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
					int(0b110111111000),
					int(-float(requestedLat) * 10 ** 7),
					int(float(requestedLon) * 10 ** 7),
					float(operatingAlt), 0, 0, 0, 0, 0, 0, brg, 0.5))

			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"GOTO ACK:  {msg}")

		except:
			traceback.print_exc()

		try:

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1,float(operatingSpeed),0,0,0,0,0,)

			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"SPD ACK:  {msg}")

		except:
			traceback.print_exc()


		await releaseSticks()

		#cancel resume
		savedLat = None
		savedLon = None

		log.info(" going to ")
		return "OK"

	finally:
		unlockV()

async def setHome(data):
	global the_connection
	global log
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
				int(-float(requestedLat) * 10 ** 7),
				int(float(requestedLon) * 10 ** 7),0,)

		msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
		log.info(f"SETHOME ACK:  {msg}")

		log.info(" rehoming")

		return "OK"

	finally:
		unlockV()

		
async def alt(data):

	global operatingAlt
	global requestedLat
	global requestedLon
	global log
	
	lockV()
	try:
		log.info("ALT")
		
		parameters = data['command']['parameters']
		
		for i in parameters:
			if i['name'] == "alt":
				operatingAlt = i['value']

				changeAlt(requestedLat, requestedLon, operatingAlt)
		
		log.info("new operating alt is now " + operatingAlt)
		return "OK"

	finally:
		unlockV()
		
async def altAdjust(delta):

	global operatingAlt
	global requestedLat
	global requestedLon
	global log
	
	lockV()
	try:
		operatingAlt = str(max(int(operatingAlt) + delta, 0))
		
		await changeAlt(requestedLat, requestedLon, operatingAlt)

		log.info("adjusted operating alt is now " + operatingAlt)

		return "OK"

	finally:
		unlockV()		

async def setToCurrAlt(data):
	global operatingAlt
	global requestedLat
	global requestedLon
	global globalPos
	global log


	lockV()
	try:
		log.info("SETCURRALT")

		operatingAlt = str(max(int(globalPos.relative_alt), 0))

		await changeAlt(requestedLat, requestedLon, operatingAlt)

		log.info("adjusted operating alt is now " + operatingAlt)

		return "OK"

	finally:
		unlockV()

	
async def changeAlt(requestedLat, requestedLon, relAlt):

	global the_connection
	global home
	global operatingAlt
	global operatingSpeed

	
	if requestedLat != None and requestedLon != None:
		#wont work in LOITER mode


		try:

			the_connection.mav.command_long_send(
				the_connection.target_system, the_connection.target_component,
				mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE,
				0,float(operatingAlt),
				mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,0,0,0,0,0,)

			msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
			log.info(f"ALT ACK:  {msg}")

		except:
			traceback.print_exc()



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
				operatingSpeed = i['value']

				the_connection.mav.command_long_send(
					the_connection.target_system, the_connection.target_component,
					mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1,float(operatingSpeed),0,0,0,0,0,)

				msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
				log.info(f"SPD ACK:  {msg}")

		
		log.info(" operating speed is now " + operatingSpeed)
		return "OK"

	finally:
		unlockV()
		
async def speedAdjust(delta):
	global the_connection
	global operatingSpeed
	global log
	
	lockV()
	try:
		operatingSpeed = str(max(min(int(operatingSpeed) + delta, 20), 1))

		the_connection.mav.command_long_send(
			the_connection.target_system, the_connection.target_component,
			mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1,float(operatingSpeed),0,0,0,0,0,)

		msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
		log.info(f"SPD ACK:  {msg}")
		
		log.info(" operating speed is now " + operatingSpeed)
		
		return "OK"

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


#override channels - center 
async def centerSticks():
	global the_connection


	the_connection.mav.send(
		mavutil.mavlink.MAVLink_rc_channels_override_message(
				the_connection.target_system, the_connection.target_component,
				1500,
				1500,
				1500,
				1500,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0)
			)
	
	msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
	log.info(f"RC CHANNELS:  {msg}")

	
#remove channel overrides
async def releaseSticks():
	global the_connection

	the_connection.mav.send(
		mavutil.mavlink.MAVLink_rc_channels_override_message(
				the_connection.target_system, the_connection.target_component,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0,
				0)
			)
	
	msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
	log.info(f"RC CHANNELS:  {msg}")

	


