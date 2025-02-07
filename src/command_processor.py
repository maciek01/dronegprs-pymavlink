#!/usr/bin/env python3

import os
import json
import time
import datetime
import sys, traceback

import pilot, video_manager

import asyncio


commandQueue = None

task = None

log = None

########################## ACTION HANDLERS #####################################

async def none(data):
	log("NONE")

async def takeoff(data):
	return await pilot.takeoff(data)

async def land(data):
	return await pilot.land(data)

async def rtl(data):
	return await pilot.rtl(data)	

async def arm(data):
	return await pilot.arm(data)	

async def disarm(data):
	return await pilot.disarm(data)	

async def position(data):
	return await pilot.position(data)	
	
async def pause(data):
	return await pilot.pause(data)
	
async def resume(data):
	return await pilot.resume(data)
	
async def loiter(data):
	return await pilot.loiter(data)

async def auto(data):
	return await pilot.auto(data)
		
async def manual(data):
	return await pilot.manual(data)
		
async def goto(data):
	return await pilot.goto(data)

async def alt(data):
	return await pilot.alt(data)
	
async def speed(data):
	return await pilot.speed(data)	

async def setToCurrAlt(data):
	return await pilot.setToCurrAlt(data)

async def decAlt1(data):
	return await pilot.decAlt1(data)
	
async def decAlt10(data):
	return await pilot.decAlt10(data)
	
async def incAlt10(data):
	return await pilot.incAlt10(data)
	
async def incAlt1(data):
	return await pilot.incAlt1(data)
	
async def decSpeed1(data):
	return await pilot.decSpeed1(data)
	
async def decSpeed10(data):
	return await pilot.decSpeed10(data)
	
async def incSpeed10(data):
	return await pilot.incSpeed10(data)
	
async def incSpeed1(data):
	return await pilot.incSpeed1(data)

async def setHome(data):
	return await pilot.setHome(data)

async def reHome(data):
	return await pilot.reHome(data)

async def toggleVid(data):
	return video_manager.toggleVid(data)

async def upWifi(data):
	return _upWifi(data)

async def downWifi(data):
	return _downWifi(data)
	
async def httpFS(data):
	return await pilot.rtl(data)

######################### ACTIONS ##############################################

actions = {
	None : none,
	"" : none,
	"NONE" : none,
	"ARM" : arm,
	"DISARM" : disarm,
	"TAKEOFF" : takeoff,
	"LAND" : land,
	"POSITION" : position,
	"PAUSE" : pause,
	"RESUME" : resume,
	"MANUAL" : manual,
	"LOITER" : loiter,
	"AUTO" : auto,
	"RTL" : rtl,
	"REHOME" : reHome,
	"TOGGLEVID" : toggleVid,
	"UPWIFI" : upWifi,
	"DOWNWIFI" : downWifi,
	"SETHOME" : setHome,
	"GOTO" : goto,
	"ALT" : alt,
	"SPEED" : speed,
	"SETCURRALT" : setToCurrAlt,
	"DECALT1" : decAlt1,
	"DECALT10" : decAlt10,
	"INCALT10" : incAlt10,
	"INCALT1" : incAlt1,
	"DECSPEED1" : decSpeed1,
	"DECSPEED10" : decSpeed10,
	"INCSPEED10" : incSpeed10,
	"INCSPEED1" : incSpeed1,
	"FS_HTTP" : httpFS
}

################################# LOCAL HANDLERS ###############################

def _upWifi(data):
	return os.system("sudo ifconfig wlan0 up")

def _downWifi(data):
	return os.system("sudo ifconfig wlan0 down")

################################# MAIN THREAD ##################################
async def processCommands():

	global commandQueue
	global actions
	global log

	while True:
		try:
			log.info("WAIT FOR ACTION:")
			action = await commandQueue.get()
			result = await actions[action['command']['name']](action)

		except Exception as inst:
			noop = None
			traceback.print_exc()
		finally:
			commandQueue.task_done()

################################ INIT ##########################################

async def processorinit(_log):
	global commandQueue
	global task
	global log

	log = _log

	commandQueue = asyncio.Queue()

	task = asyncio.create_task(coro=processCommands(), name="command_processor")


