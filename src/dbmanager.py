#!/usr/bin/env python



import sqlite3 as lite
import sys

con = None
log = None


def open(dbname):
	global con
	global log

	try:
		con = lite.connect(dbname)
	except lite.Error as e:
		log.info("Error {}:".format(e.args[0]))

def close():
	global con
	global log

	if con != None:
		try:
			con.close()
		except lite.Error as e:
			log.info("Error {}:".format(e.args[0]))

def getWaypoints():
	global log

	waypoints = None

	try:
		cur = con.cursor()

		cur.execute("SELECT name, lat, lon from waypoints")
		waypoints = cur.fetchall()
	except lite.Error as e:
		log.info("Error {}:".format(e.args[0]))

	return waypoints




