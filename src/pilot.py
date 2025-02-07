#!/usr/bin/env python3


import sys, traceback
import threading
import time, datetime, json
import asyncio
from mavsdk import System
import numpy
import math


current_milli_time = lambda: int(time.time() * 1000)

task = None

operatingAlt = 100
operatingSpeed = 15
requestedLat = None
requestedLon = None
savedLat = None
savedLon = None

vehicle = None
vehicleLock = threading.RLock()
URL = None
BAUD = None
log = None

#monitoring tasks

tasks = []

#MAVLINK status
curr_tot = 0
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
connected = False



########################### STATE AND MESSAGE OBSERVERS ########################


async def onPX4_mode(drone):
	global mode
	async for my_mode in drone.telemetry.flight_mode():
		mode = str(my_mode)

async def onPX4_battery(drone):
	global curr_tot
	global voltages
	global bat_percent
	async for battery in drone.telemetry.battery():
		bat_percent = battery.remaining_percent
		voltages = battery.voltage_v

async def onPX4_heading(drone):
	global heading
	async for head in drone.telemetry.heading():
		heading = head.heading_deg

async def onPX4_statusText(drone):
	global statusMessage
	global statusSev
	async for status_text in drone.telemetry.status_text():
		statusMessage = status_text.text
		statusSev = status_text.type

async def onPX4_gps_info(drone):
	global gps_info
	async for my_gps_info in drone.telemetry.gps_info():
		gps_info = my_gps_info

async def print_in_air(drone):
	async for in_air in drone.telemetry.in_air():
		print(f"In air: {in_air}")

async def onPX4_position(drone):
	global pos
	async for my_position in drone.telemetry.position():
		pos = my_position

async def onPX4_raw_gps(drone):
        global gps_data
        async for my_gps in drone.telemetry.raw_gps():
                gps_data = my_gps

async def onPX4_home(drone):
        global home
        async for my_home in drone.telemetry.home():
                home = my_home

async def onPX4_is_armed(drone):
	global armed
	async for is_armed in drone.telemetry.armed():
		armed = is_armed



########################### THREAD HELPERS #####################################

def lockV():
	vehicleLock.acquire()

def unlockV():
	vehicleLock.release()

async def initVehicle():
	global URL
	global BAUD
	global vehicle
	global tasks
	global log


	lockV()
	try:
		if vehicle != None:
			for task in tasks:
				try:
					task.cancel()
				except:
					log.info("ERROR WHEN CANCELLING A TASK")

			tasks = []
			#close vehicle
			try:
				vehicle.close()
				vehicle = None
			except Exception as inst:
				vehicle = None

		#open vehicle

		while vehicle == None:
			try:
				# Init the drone

				#PX4
				_vehicle = System()
				#await _vehicle.connect(system_address="udp://:14540")
				log.info("calling connect " + URL)
				await _vehicle.connect(system_address=URL)
				log.info("connect accepted " + URL)

				log.info("Waiting for drone to connect...")
				async for state in _vehicle.core.connection_state():
					if state.is_connected:
						connected = True
						log.info(f"Connected to vehicle")
						break
				log.info("Connected via " + URL)
				vehicle = _vehicle


			except Exception as inst:
				vehicle = None
				traceback.print_exc()
				await asyncio.sleep(5)


		#register listeners

		await vehicle.telemetry.set_rate_position(1)
		await vehicle.telemetry.set_rate_in_air(1)
		await vehicle.telemetry.set_rate_landed_state(1)
		await vehicle.telemetry.set_rate_battery(5)
		await vehicle.telemetry.set_rate_gps_info(1)
		await vehicle.telemetry.set_rate_home(5)


		tasks.append(asyncio.create_task(onPX4_battery(vehicle)))
		tasks.append(asyncio.create_task(onPX4_mode(vehicle)))
		tasks.append(asyncio.create_task(onPX4_heading(vehicle)))
		tasks.append(asyncio.create_task(onPX4_statusText(vehicle)))
		tasks.append(asyncio.create_task(onPX4_gps_info(vehicle)))
		#tasks.append(asyncio.create_task(print_in_air(vehicle)))
		tasks.append(asyncio.create_task(onPX4_position(vehicle)))
		#asyncio.ensure_future(onPX4_position(vehicle))
		tasks.append(asyncio.create_task(onPX4_raw_gps(vehicle)))
		tasks.append(asyncio.create_task(onPX4_home(vehicle)))
		tasks.append(asyncio.create_task(onPX4_is_armed(vehicle)))



	finally:
		unlockV()



####################### MAIN THREAD ############################################

async def pilotMonitor():

	global vehicle
	global log

	#wait to initialize the pilot
	log.info("ABOUT to call INIT VEH")

	await initVehicle()

	#read loop

	while True:
		try:
			await asyncio.sleep(1)

			async for state in vehicle.core.connection_state():
				connected = state.is_connected
				if not state.is_connected:
					log.info(f"Disconnected from vehicle")
					break

			#if not connected:
			#	log.info("REINIT VEHICLE CONNECTION")
			#	await initVehicle()

		except Exception as inst:
			#traceback.print_exc()
			await initVehicle()



###################### INIT HANDLER ############################################
	
async def pilotinit(_log, url, baud):
	global URL
	global BAUD
	global task
	global log

	log = _log

	URL = url
	BAUD = baud

	task = asyncio.create_task(coro=pilotMonitor(), name="pilot")

############################ LOACAL FUNCs     ##################################

def get_bearing(lat1, long1, lat2, long2):
	dLon = (long2 - long1)
	x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
	y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
	brng = numpy.arctan2(x,y)
	brng = numpy.degrees(brng)

	return brng



############################ COMMAND HANDLERS ##################################

async def arm(data):

	global vehicle
	global log

	lockV()

	try:
		log.info("ARM")

		try:
			await vehicle.action.arm()
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

	global vehicle
	global log

	lockV()
	try:
		log.info("DISARM")
			
		try:
			await vehicle.action.disarm()
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


async def takeoff(data):

	global vehicle
	global log

	lockV()
	try:
	
		aTargetAltitude = operatingAlt

		#request and wait for the arm thread to be armed	
		try:
			await arm(data)
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
			
		await vehicle.action.set_takeoff_altitude(float(aTargetAltitude))
		try:
			await vehicle.action.takeoff()
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

	global vehicle
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
			await vehicle.action.land()
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

	global vehicle
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
			await vehicle.action.hold()
		except:
			traceback.print_exc()
		
		#save last goto
		savedLat = requestedLat
		savedLon = requestedLon
		requestedLat = None
		requestedLon = None
		
			
		return "OK"

	finally:
		unlockV()
		

async def loiter(data):

	global vehicle
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
			await vehicle.action.hold()
		except:
			traceback.print_exc()
		
		#save last goto
		savedLat = requestedLat
		savedLon = requestedLon
		requestedLat = None
		requestedLon = None
		
			
		return "OK"

	finally:
		unlockV()		

async def auto(data):

	global vehicle
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
			await vehicle.mission.start_mission()
		except:
			traceback.print_exc()
		
		#save last goto
		savedLat = None
		savedLon = None
		requestedLat = None
		requestedLon = None
		
			
		return "OK"

	finally:
		unlockV()
		
async def pause(data):

	global vehicle
	global log
	
	lockV()
	try:
		log.info("PAUSE")
			
		
		if True:
			#just loiter
			try:
				await vehicle.action.hold()
			except:
				traceback.print_exc()

		
			
		return "OK"

	finally:
		unlockV()

async def resume(data):

	global vehicle
	global pos
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
			#vehicle.mode = VehicleMode("GUIDED")


			brg = get_bearing(pos.latitude_deg, pos.longitude_deg, float(requestedLat), float(requestedLon))


			try:
				await vehicle.action.goto_location(float(requestedLat), float(requestedLon), float(home.absolute_altitude_m) + float(operatingAlt), float(brg))
			except:
				traceback.print_exc()

			try:
				await vehicle.action.set_current_speed(float(operatingSpeed))
			except:
				traceback.print_exc()


			savedLat = None
			savedLon = None
			await releaseSticks()
		
			
		return "OK"

	finally:
		unlockV()

#release channel overrides
async def manual(data):

	global vehicle
	global log
	
	lockV()
	try:
		log.info("MANUAL")
		try:
			await vehicle.action.hold()
		except:
			traceback.print_exc()

		await releaseSticks()
		return "OK"

	finally:
		unlockV()

async def reHome(data):
	global vehicle
	global log
	
	lockV()
	try:
		log.info("REHOME - not supported")

		#vehicle.home_location=vehicle.location.global_frame

		return "OK"

	finally:
		unlockV()
		
async def rtl(data):

	global vehicle
	global requestedLat
	global requestedLon
	global operatingSpeed
	global savedLat
	global savedLon	   
	global log
	
	lockV()
	try:
		log.info("RTL")
		#if not armed:
			#print " NOT ARMED"
			#return "ERROR: NOT ARMED"

		try:
			await vehicle.action.return_to_launch()
		except:
			traceback.print_exc()

		try:
			await vehicle.action.set_current_speed(float(operatingSpeed))
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

	global vehicle
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

		brg = get_bearing(pos.latitude_deg, pos.longitude_deg, float(requestedLat), float(requestedLon))


		try:
			await vehicle.action.goto_location(float(requestedLat), float(requestedLon), float(home.absolute_altitude_m) + float(operatingAlt), float(brg))
		except:
			traceback.print_exc()

		try:
			await vehicle.action.set_current_speed(float(operatingSpeed))
		except:
			traceback.print_exc()


		await releaseSticks()

		#cancel resume
		savedLat = None
		savedLon = None

		log.info("going to ")
		return "OK"

	finally:
		unlockV()

async def setHome(data):
	global vehicle
	global log
	lockV()
	try:
		log.info("SETHOME - not supported")
		
		parameters = data['command']['parameters']
		
		for i in parameters:
			if i['name'] == "lat":
				lat = i['value']
			if i['name'] == "lon":
				lon = i['value']
		
		#point1 = LocationGlobal(float(lat), float(lon), vehicle.home_location.alt)
		#if vehicle.home_location != None:
		#	vehicle.home_location=point1

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
	global pos
	global log


	lockV()
	try:
		log.info("SETCURRALT")

		operatingAlt = str(max(int(pos.relative_altitude_m), 0))

		await changeAlt(requestedLat, requestedLon, operatingAlt)

		log.info("adjusted operating alt is now " + operatingAlt)

		return "OK"

	finally:
		unlockV()

	
async def changeAlt(requestedLat, requestedLon, relAlt):

	global vehicle
	global pos
	global home
	global operatingAlt
	global operatingSpeed

	
	if requestedLat != None and requestedLon != None:
		#wont work in LOITER mode

		brg = get_bearing(pos.latitude_deg, pos.longitude_deg, float(requestedLat), float(requestedLon))


		try:
			await vehicle.action.goto_location(float(requestedLat), float(requestedLon), float(home.absolute_altitude_m) + float(operatingAlt), float(brg))
		except:
			traceback.print_exc()

		try:
			await vehicle.action.set_current_speed(float(operatingSpeed))
		except:
			traceback.print_exc()


async def speed(data):

	global vehicle
	global operatingSpeed
	global log
	
	lockV()
	try:
		log.info("SPEED")
		
		parameters = data['command']['parameters']
		
		for i in parameters:
			if i['name'] == "speed":
				operatingSpeed = i['value']
				await vehicle.action.set_current_speed(float(operatingSpeed))
		
		log.info(" operating speed is now " + operatingSpeed)
		return "OK"

	finally:
		unlockV()
		
async def speedAdjust(delta):
	global vehicle
	global operatingSpeed
	global log
	
	lockV()
	try:
		operatingSpeed = str(max(min(int(operatingSpeed) + delta, 20), 1))
		await vehicle.action.set_current_speed(float(operatingSpeed))
		
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
	global vehicle

	#await vehicle.manual_control.set_manual_control_input(
	#	float(0), float(0), float(0.5), float(0)
	#)

	#await vehicle.manual_control.start_position_control()
	
#remove channel overrides
async def releaseSticks():
	global vehicle
	#await vehicle.manual_control.set_manual_control_input(
	#	float(0), float(0), float(0.5), float(0)
	#)
	#await vehicle.action.hold()

	#vehicle.channels.overrides = {}
	


