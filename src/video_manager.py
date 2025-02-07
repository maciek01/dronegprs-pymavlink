#!/usr/bin/env python

import subprocess, time, os, signal
import Main

stream_cmd = None

process = None


def init(cmd):
	global stream_cmd

	stream_cmd = cmd


def toggleVid(data):

	global stream_cmd
	global process

	if process != None:

#		try:
#			process.terminate()
#		except Exception as inst:
#			Main.log.info( inst
#
#		time.sleep(1)
#
#		try:
#			process.kill()
#		except Exception as inst:
#			Main.log.info( inst

		try:
			os.killpg(os.getpgid(process.pid), signal.SIGTERM)
		except Exception as inst:
			Main.log.info(inst)

		process = None

	else:

		try:
			process = subprocess.Popen(stream_cmd, shell=True, stdout=subprocess.PIPE, preexec_fn=os.setsid)
			#process.wait()
			#print process.returncode
		except Exception as inst:
			process = None


	return None


