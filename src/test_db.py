#!/usr/bin/env python

import logger
import dbmanager


dbmanager.log = logger.setup_custom_logger('main')

dbmanager.open("/home/pi/uavonboard.db")

wps = dbmanager.getWaypoints();

dbmanager.close()


for wp in wps:
	print(wp[0], wp[1], wp[2])



