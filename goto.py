#!/usr/bin/env python3
# takeoff.py
#https://github.com/Intelligent-Quads/iq_pymavlink_tutorial/tree/master

import argparse
from pymavlink import mavutil
import time



def connect_to_sysid(connection_str : str, sysid : int, timeout: float = 3) -> any:
    """
    connect_to_sysid connects to a mavlink stream with a specific sysid

    Args:
        connection_str (str): String containing the connection information
        sysid (int): The system id to connect to
        timeout (float, optional): Maximum time to wait for the connection in seconds. Defaults to 3.

    Returns:
        mavutil.mavlink_connection: Returns the connection object if connection is successful, 
        else returns None after the timeout
    """    
    the_connection = mavutil.mavlink_connection(connection_str)
    time_start = time.time()

    while time.time() - time_start < timeout:
        try:
            the_connection.wait_heartbeat()
            print(f"Heartbeat from system {the_connection.target_system} component {the_connection.target_component}")
            if the_connection.target_system == sysid:
                print(f"Now connected to SYSID {sysid}")
                return the_connection
        except Exception as e:
            print(f"Error while waiting for heartbeat: {e}")
            return None

    print(f"Connection timeout after {timeout} seconds")
    return None

def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id=1):

    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    #wait_until_position_aiding(mav_connection)


    print("Connected to ArduPilot autopilot")
    mode_id = mav_connection.mode_mapping()["GUIDED"]
    takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    # Arm the UAS
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

    # Command Takeoff
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3],
        takeoff_params[4], takeoff_params[5], takeoff_params[6])

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")

    return takeoff_msg.result

def main():
    parser = argparse.ArgumentParser(description="A simple script to command a UAV to takeoff.")
    parser.add_argument("--altitude", type=int, help="Altitude for the UAV to reach upon takeoff.", default=10)
    parser.add_argument("--sysid", type=int, help="System ID of the UAV to command.", default=1)


    args = parser.parse_args()
    mav_connection = connect_to_sysid('udpin:localhost:14551', args.sysid)
    takeoff(mav_connection, args.altitude)

    time.sleep (15)

    mav_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
            10, mav_connection.target_system,
            mav_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            int(0b110111111000),
            int(-35.3629849 * 10 ** 7),
            int(149.1649185 * 10 ** 7),
            10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))


    while 1:
        msg = mav_connection.recv_match(
            #type='LOCAL_POSITION_NED', blocking=True)
            blocking=True)
        print(msg)

    

if __name__ == "__main__":
    main()