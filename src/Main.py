#!/usr/bin/env python3

import httplib2
import requests
import time
import json
import socket
import sys
import traceback
import gps, pilot, modem, video_manager
import command_processor
import argparse
import configparser
import dbmanager
import logger
import asyncio

HOST = "home.kolesnik.org"

#HTTP FAILSAFE SUPPORT
fs_http_triggered = False

#heartbeat sucess ts for FAILSAFE
good_heartbeat = None
#good_heartbeat = pilot.current_milli_time()

#1 minute before triggerting http FS
FS_TRESHOLD = 60000

#request timeout
HTTP_TIMEOUT = 5

gpsPort = None
unitID = None
videoChannel = None
mavlinkPort = None

log = None


def subst(str, net = False):
	global HOST
	global unitID
	ipAddress = HOST
	if net:
		try:
			ipAddress = socket.gethostbyname(HOST)
		except Exception:
			ipAddress = HOST

	if str != None:
		str = str.replace("${HOSTNAME}", HOST)
		str = str.replace("${unitID}", unitID)
		if net:
			str = str.replace("${HOSTIP}", ipAddress)

	return str

#this is for GPS pinger mode if enabled
def reportGPSData():
	global unitID
	global videoChannel
	global gpsPort

	if gpsPort == "":
		return None

	data = {
		"unitId" : unitID,
		"videoChannel" : videoChannel,
		"stateTimestampMS" : gps.current_milli_time(),
		"gpsLatLon" : gps.GPSLAT + " / " + gps.GPSLON,
		"gpsLat" : gps.GPSLATNORM,
		"gpsLon" : gps.GPSLONNORM,
		"gpsAlt" : gps.GPSALT,
		"gpsAltRel" : gps.GPSALT,
		"gpsSpeed" : gps.GPSSPEED,
		"heading" : gps.GPSHEADING,
		"gpsTime" : gps.GPSTIME,
		"gpsStatus" : gps.GPSSTATUS,
		"gpsNumSats" : gps.GPSNSATS,
		"gpsLock" : gps.GPSFIX,
		"gpsLastStatusMS" : gps.GPSLASTSTATUSMS,
		"unitCallbackPort" : "8080",
		"modemstatus" : modem.MODEMSTATUS,
		"modemsignal" : modem.MODEMSIGNAL

	}

	return data

#this is for mission control mode if enabled
def reportPilotData():
	global unitID
	global videoChannel
	global mavlinkPort

	if pilot.the_connection == None or mavlinkPort == "":
		return None
	
	try:
		pilot.gpsRaw
		gps1 = pilot.gpsRaw
	except Exception:
		gps1 = None
	
	try:
		pilot.gpsRaw
		gpsd = pilot.gpsRaw
	except Exception:
		gpsd = None
	
	try:
		pilot.globalPos
		posGlobal = pilot.globalPos
	except Exception:
		posGlobal = None

	data = {
		#1s reporting
		"unitId" : unitID,
		"videoChannel" : videoChannel,

		"stateTimestampMS" : pilot.current_milli_time(),
		"gpsLatLon" : "",
		"gpsLat" : posGlobal.lat / 10 ** 7 if posGlobal != None else None,
		"gpsLon" : posGlobal.lon / 10 ** 7 if posGlobal != None else None,
		"gpsAlt" : posGlobal.alt / 1000 if posGlobal != None else None,
		"gpsAltRel" : posGlobal.relative_alt / 1000 if posGlobal != None else None,

		"homeLatLon" : "",
		"homeLat" : pilot.home.latitude / 10 ** 7 if pilot.home != None else None,
		"homeLon" : pilot.home.longitude / 10 ** 7 if pilot.home != None else None,
		"homeAlt" : pilot.home.altitude / 1000 if pilot.home != None else None,
		
		"operatingAlt" : pilot.operatingAlt,
		"operatingSpeed" : pilot.operatingSpeed,

		"gpsSpeed" : gpsd.vel / 100  if gpsd != None else None,
		"gpsTime" : gpsd.time_usec if gpsd != None else None,
		"gpsStatus" : "none",
		#"gpsLastStatusMS" : pilot.current_milli_time() - pilot.vehicle.last_heartbeat,
		"gpsLastStatusMS" : "N/A",

		"airSpeed" : gpsd.vel / 100 if gpsd != None else None,
		#"heading" : gpsd.yaw_deg if gpsd != None else None,
		"heading" : pilot.globalPos.hdg / 100 if pilot.globalPos != None else None,
		"cog" : gpsd.cog / 100 if gpsd != None else None,
		"baroAlt" : posGlobal.alt / 1000 if posGlobal != None else None,
		"sonarAlt" : posGlobal.relative_alt / 1000 if posGlobal != None else None,
		"lidarAlt" : posGlobal.relative_alt / 1000 if posGlobal != None else None,
		#"status" : pilot.vehicle.system_status.state,
		"status" : "N/A",
		"mode" : pilot.flightMode,
		"armed" : pilot.armed,

		#5 sec reporting
		"gpsNumSats" : gps1.satellites_visible if gps1 != None else None,
		"gpsLock" : str(gps1.fix_type) if gps1 != None else None,
		"gpsHError" : gpsd.eph if gpsd != None else None,
		"gpsVError" : gpsd.epv if gpsd != None else None,

		"gps2NumSats" : gps1.satellites_visible if gps1 != None else None,
		"gps2Lock" : str(gps1.fix_type) if gps1 != None else None,
		"gps2HError" : gpsd.eph if gpsd != None else None,
		"gps2VError" : gpsd.epv if gpsd != None else None,

		"currVolts" : pilot.sysstatus.voltage_battery / 1000 if pilot.sysstatus != None else None,
		"currVoltsLevel" : pilot.battery.battery_remaining if pilot.battery != None else None,
		"currA" : pilot.battery.current_battery / 100 if pilot.battery != None else None,
		"currTotmAh" : pilot.battery.current_consumed if pilot.battery != None else None,
		"voltages" : pilot.battery.voltages if pilot.battery != None else None,

		"videostat" : "ON" if video_manager.process != None else "OFF",

		"modemstatus" : modem.MODEMSTATUS,
		"modemsignal" : modem.MODEMSIGNAL,

		"message" : pilot.statustext.text if pilot.statustext != None else None,
		"messageSev" : str(pilot.statustext.severity) if pilot.statustext != None else None,

		#30 s reporting
		"unitCallbackPort" : "8080"
		}

	return data

#combine both sources to unified model - pilotData
def mergeData(pilotData, gpsData):

	if pilotData == None:
		return gpsData
	if gpsData == None:
		return pilotData

	pilotData["gpsLatLon"] = gpsData["gpsLatLon"]
	pilotData["gpsTime"] = gpsData["gpsTime"]
	pilotData["gpsStatus"] = gpsData["gpsStatus"]

	return pilotData

async def sendHeartbeat(log, unitID, videoChannel, http, url, headers):

	content = None
	global good_heartbeat

	try:
		gpsData = reportGPSData()
		data = reportPilotData()
		data = mergeData(data, gpsData)
		modem.pilotData = data
		if data != None:
			log.info("sending heartbeat")
			try:
				response, content = http.request( url, 'POST', json.dumps(data), headers=headers)
				content = content.decode("utf-8")
				#response = requests.post(url, json = data, headers = headers, timeout = HTTP_TIMEOUT)
				#content = response.content
				#response.close()
				log.info("heartbeat sent")
				good_heartbeat = pilot.current_milli_time()
			except Exception as inst:
				log.error("http timeout")
				traceback.print_exc()
		else:
			log.info("nothing to send")
			data = {
				#1s reporting
				"unitId" : unitID,
				"videoChannel" : videoChannel,
				"videostat" : "ON" if video_manager.process != None else "OFF",

				"modemstatus" : modem.MODEMSTATUS,
				"modemsignal" : modem.MODEMSIGNAL,

				#30 s reporting
				"unitCallbackPort" : "8080"
			}
			log.info("sending heartbeat")
			try:
				response, content = http.request( url, 'POST', json.dumps(data), headers=headers)
				content = content.decode("utf-8")
				#response = requests.post(url, json = data, headers = headers, timeout = HTTP_TIMEOUT)
				#content = response.content
				#response.close()
				log.info("heartbeat sent")
				good_heartbeat = pilot.current_milli_time()
			except Exception as inst:
				log.error("http timeout")
				traceback.print_exc()


	except Exception as inst:
		noop = None
		#comment out the following if runnign as daemon
		traceback.print_exc()
		return

	try:
		if content != None and content != "":
			actions = json.loads(content)
			log.info("COMMANDS:" + content)
			if actions != None and actions['data'] != None and actions['data']['actionRequests'] != None:
				log.info("actionRequests: " + json.dumps(actions['data']['actionRequests']))
				for i in actions['data']['actionRequests']:
					await command_processor.commandQueue.put(i)

	except Exception as inst:
		noop = None
		traceback.print_exc()


#main run
async def run():

	global fs_http_triggered
	global HOST
	global fs_http_triggered
	global good_heartbeat
	global FS_TRESHOLD
	global HTTP_TIMEOUT
	global gpsPort
	global unitID
	global videoChannel
	global mavlinkPort
	global log

	log = logger.setup_custom_logger('main')

	log.info("STARTING MAIN MODULE")

	httplib2.debuglevel     = 0
	http                    = httplib2.Http(timeout=HTTP_TIMEOUT)
	content_type_header     = "application/json"

	#parse args	

	parser = argparse.ArgumentParser()
	parser.add_argument("--config", default="/home/pi/main.cfg")
	args = parser.parse_args()

	cfg = args.config

	#read config

	config = configparser.ConfigParser()
	#config.readfp(open(cfg, 'r'))
	config.read(cfg)

	#read cfg params
	HOST = config.get('main', 'HOSTNAME')
	HOST = HOST if HOST != "" else "home.kolesnik.org"
	unitID = config.get('main', 'unitID')
	unitID = unitID if unitID != "" else "uav0"
	videoChannel = subst(config.get('main', 'videoChannel'))
	mavlinkPort = subst(config.get('main', 'mavlinkPort'))
	mavlinkBaud = subst(config.get('main', 'mavlinkBaud'))
	gpsPort = subst(config.get('main', 'gpsPort'))
	gpsBaud = subst(config.get('main', 'gpsBaud'))
	modemPort = subst(config.get('main', 'modemPort'))
	modemBaud = subst(config.get('main', 'modemBaud'))
	modems = subst(config.get('main', 'modems'))
	host = subst(config.get('main', 'host'))
	uri = subst(config.get('main', 'uri'))
	dbfile = subst(config.get('main', 'dbfile'))
	FS_TRESHOLD = int(config.get('main', 'FS_TRESHOLD'))
	videoStreamCmd = subst(config.get('main', 'videoStreamCmd'))

	#apply cfg defaults

	dbfile = dbfile if dbfile != "" else "/home/pi/uavonboard.db"
	unitID = unitID if unitID != "" else "uav0"
	uri = uri if uri != "" else "/uavserver/v1/heartbeat"
	host = host if host != "" else "http://home.kolesnik.org:8000"
	url = host + uri

	log.info("CONFIGURATION:")
	log.info(" unitID:" + unitID)
	log.info(" videoChannel:" + videoChannel)
	log.info(" url:" + url)
	log.info(" mavlinkPort:" + mavlinkPort)
	log.info(" mavlinkBaud:" + mavlinkBaud)
	log.info(" gpsPort:" + gpsPort)
	log.info(" gpsBaud:" + gpsBaud)
	log.info(" modemPort:" + modemPort)
	log.info(" modemBaud:" + modemBaud)
	log.info(" modems:" + modems)
	log.info(" dbfile:" + dbfile)
	log.info(" FS_TRESHOLD:" + str(FS_TRESHOLD))
	log.info(" videoStreamCmd:" + videoStreamCmd)


	headers = {"Content-Type": content_type_header}

	#initialize database
	log.info("STARTING DATABASE " + dbfile)
	dbmanager.open(dbfile)

	#initialize modem monitor
	modem.loginit(log)
	if modemPort == "" and modems != "":
		log.info("LOOK FOR AVAILABLE MODEMS ...")
		modemList = modems.split(',')
		firstModem = await modem.findModem(modemList, int(modemBaud))
	else:
		firstModem = modemPort

	if firstModem != "":
		log.info("STARTING MODEM MODULE AT " + firstModem)
		await modem.modeminit(firstModem, int(modemBaud), 5, True)

	#net context available
	videoStreamCmd = subst(config.get('main', 'videoStreamCmd'), net=True)


	#initialize pilot
	if mavlinkPort != "":
		log.info("STARTING PILOT MODULE AT " + mavlinkPort)
		await pilot.pilotinit(log, mavlinkPort, int(mavlinkBaud))

	#initialize gps
	if gpsPort != "":
		log.info("STARTING GPS MODULE AT " + gpsPort)
		gps.gpsinit(gpsPort, int(gpsBaud))

	#initialize video streaming
	video_manager.init(videoStreamCmd)

	log.info("STARTING COMMAND PROCESSOR MODULE")
	#initialize command queue
	await command_processor.processorinit(log)



	#wait for vehicle connection
	while pilot.the_connection == None and mavlinkPort != "":
		log.info(" Waiting for vehicle connection ...")
		await asyncio.sleep(1)
		await sendHeartbeat(log, unitID, videoChannel, http, url, headers)
		if good_heartbeat != None:
			if pilot.current_milli_time() - good_heartbeat > FS_TRESHOLD:
				log.info("FAILSAFE - noncritical")

	#pilot.the_connection.wait_heartbeat()
	#log.info("Main: Connected")

	#wait for health and home
	while pilot.home == None:
		log.info(f"Waiting for HOME location and good health")
		await sendHeartbeat(log, unitID, videoChannel, http, url, headers)
		await asyncio.sleep(1)
		#time.sleep(1)

	log.info("Global position state is good enough for flying.")
	await sendHeartbeat(log, unitID, videoChannel, http, url, headers)

	#get home loc
	log.info(f"Home location: {pilot.home}")


	log.info("STARTING COMMAND LOOP")
	while True:
		await asyncio.sleep(1)
		await sendHeartbeat(log, unitID, videoChannel, http, url, headers)
		if good_heartbeat != None:
			if pilot.current_milli_time() - good_heartbeat > FS_TRESHOLD:
				log.error("FAILSAFE condition")
				if not fs_http_triggered:
					await command_processor.commandQueue.put(json.loads('{"command":{"name" : "FS_HTTP"}}'))
					log.error("HTTP FAILSAFE triggered")
					fs_http_triggered = True
			else:
				if fs_http_triggered:
					log.error("FAILSAFE cleared")
					fs_http_triggered = False

#main
if __name__ == '__main__':
	asyncio.run(run())

