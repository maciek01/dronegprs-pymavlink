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

async def initVehicle():

      global the_connection

      # Start a connection listening to a UDP port
      the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
      #the_connection = mavutil.mavlink_connection('/dev/serial0', 57600)

      # Wait for the first heartbeat
      #   This sets the system and component ID of remote system for the link
      the_connection.wait_heartbeat()
      print("Heartbeat from system (system %u component %u)" %
            (the_connection.target_system, the_connection.target_component))




async def run():

      global the_connection

      await initVehicle()



      while 1:
                  
            try:
                  #await asyncio.sleep(1)
                  
                  msg = the_connection.recv_match(
                        #type='LOCAL_POSITION_NED', blocking=True)
                              blocking=True)
                  print(msg)


            except Exception as e:
                  print(e)



if __name__ == "__main__":

      asyncio.run(run())
