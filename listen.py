#!/usr/bin/env python3


# https://mavlink.io/en/mavgen_python/#receiving-messages

# https://mavlink.io/en/mavgen_python/howto_requestmessages.html

# https://ardupilot.org/dev/docs/ArduCopter_MAVLink_Messages.html

# AP specific:
# https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html

import asyncio
import threading
import time
from pymavlink import mavutil





the_connection = None
gpsRaw = None
globalPos = None
home = None
battery = None
rangefinder = None
statustext = None
sysstatus = None
heartbeat = None
rcChannels = None
rcChannelsRaw = None



def setMessageFrequency(message_id, frequency):
      global the_connection
      message = the_connection.mav.command_long_encode(
            the_connection.target_system,  # Target system ID
            the_connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ID of command to send
            0,  # Confirmation
            message_id,  # param1: Message ID to be streamed
            frequency, # param2: Interval in microseconds
            0,       # param3 (unused)
            0,       # param4 (unused)
            0,       # param5 (unused)
            0,       # param5 (unused)
            0        # param6 (unused)
      )

      the_connection.mav.send(message)

      # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
      response = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
      if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Command accepted for message ID %d with frequency %d microseconds" % (message_id, frequency))
      else:
            print("Command failed for message ID %d" % message_id)


def telemMonitor():
      global the_connection
      global gpsRaw
      global globalPos
      global home
      global battery
      global rangefinder
      global statustext
      global sysstatus
      global heartbeat
      global rcChannels
      global rcChannelsRaw



      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 5000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 5000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_RANGEFINDER, 1000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT, 1000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 1000000)
      setMessageFrequency(mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS_RAW, 1000000)


      while 1:
            try:
                  time.sleep(1)
                  #print("Waiting for telem")
                  gpsRaw = the_connection.recv_match(type='GPS_RAW_INT', blocking=False)
                  globalPos = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                  battery = the_connection.recv_match(type='BATTERY_STATUS', blocking=False)
                  #mode = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                  home = the_connection.recv_match(type='HOME_POSITION', blocking=False)
                  #armed = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                  rangefinder = the_connection.recv_match(type='RANGEFINDER', blocking=False)
                  statustext = the_connection.recv_match(type='STATUSTEXT', blocking=False)
                  sysstatus = the_connection.recv_match(type='SYS_STATUS', blocking=False)
                  heartbeat = the_connection.recv_match(type='HEARTBEAT', blocking=False)
                  rcChannels = the_connection.recv_match(type='RC_CHANNELS', blocking=False)
                  rcChannelsRaw = the_connection.recv_match(type='RC_CHANNELS_RAW', blocking=False)


            except Exception as e:
                  print("error in monitor receive", e)

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
                  if rcChannels is None:
                        rcChannels = getMessage('RC_CHANNELS')
                  if rcChannelsRaw is None:
                        rcChannelsRaw = getMessage('RC_CHANNELS_RAW')


            except Exception as e:
                  print("error in monitor get", e)



def getMessage(msg):
      global the_connection

      try:
            return the_connection.messages[msg]
      except:
            return None

async def initVehicle():

      global the_connection

      # Start a connection listening to a UDP port
      the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

      # Wait for the first heartbeat
      #   This sets the system and component ID of remote system for the link
      the_connection.wait_heartbeat()
      print("Heartbeat from system (system %u component %u)" %
            (the_connection.target_system, the_connection.target_component))


      monitor_thread = threading.Thread(target=telemMonitor, args=())
      monitor_thread.daemon = True
      monitor_thread.start()


async def run():

      global the_connection
      global gpsRaw
      global globalPos
      global home
      global battery
      global rangefinder
      global statustext
      global sysstatus
      global heartbeat
      global rcChannels
      global rcChannelsRaw


      await initVehicle()


      msg = the_connection.mav.send(
            mavutil.mavlink.MAVLink_rc_channels_override_message(
                        #the_connection.target_system, the_connection.target_component,
                        0,0,
                        2000,
                        2000,
                        2000,
                        2000,
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

      #msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
      print(f"RC CHANNELS:  {msg}")

      while 1:
            await asyncio.sleep(1)

            try:
                  print("--------------------------------------------------")
                  print("CON:",
                        the_connection.flightmode,
                        the_connection.location(),
                        bool(the_connection.motors_armed())
                        )
                  print("GPS %d s:" % the_connection.time_since('GPS_RAW_INT'), gpsRaw)
                  print("RPO %d s:" % 0, the_connection.location())
                  print("POS %d s:" % the_connection.time_since('GLOBAL_POSITION_INT'), globalPos)
                  print("HME %d s:" % the_connection.time_since('HOME_POSITION'), home)
                  print("BAT %d s:" % the_connection.time_since('BATTERY_STATUS'), battery)
                  print("RNG %d s:" % the_connection.time_since('RANGEFINDER'), rangefinder)
                  print("STX %d s:" % the_connection.time_since('STATUSTEXT'), statustext)
                  print("SYS %d s:" % the_connection.time_since('SYS_STATUS'), sysstatus)
                  print("HRT %d s:" % the_connection.time_since('HEARTBEAT'), heartbeat)
                  print("RCC %d s:" % the_connection.time_since('RC_CHANNELS'), rcChannels)
                  print("RCR %d s:" % the_connection.time_since('RC_CHANNELS_RAW'), rcChannelsRaw)

            except Exception as e:
                  print(e)


def translate_flight_mode(mode):
      flight_modes = {
            0: "Stabilize",
            1: "Acro",
            2: "AltHold",
            3: "Auto",
            4: "Guided",
            5: "Loiter",
            6: "RTL",
            7: "Circle",
            9: "Land",
            11: "Drift",
            13: "Sport",
            14: "Flip",
            15: "AutoTune",
            16: "PosHold",
            17: "Brake",
            18: "Throw",
            19: "Avoid_ADSB",
            20: "Guided_NoGPS",
            21: "SmartRTL",
            22: "FlowHold",
            23: "Follow",
            24: "ZigZag",
            25: "SystemID",
            26: "Heli_Autorotate",
            27: "Auto_RTL",
            28: "Turtle"
      }
      return flight_modes.get(mode, "Unknown")

if __name__ == "__main__":

      asyncio.run(run())