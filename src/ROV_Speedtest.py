import sys

from src import helpers
from timeit import default_timer
import numpy as np
import signal
import time

def exit_func(signal, frame, master):
    print("Disarming before exiting")
    helpers.disarm(master)
    print("Exiting")
    sys.exit(0)

def run(conn_type:str="SC2A"):
    """
    emulates joystick commands and slowly ramps them up

    *DO NOT* run thrusters for longer than 30 seconds in air or you will wear out the plastic bearings.
    Source: https://bluerobotics.com/learn/bluerov2-software-setup/

    Here: 3 * 2s per direction and 4 different directions = 24s (plus sleep time)
    """
    print("ROV Speedtest")
    print("'Ctrl + C' to disarm the BlueROV and exit the script")
    time.sleep(2)

    print("\nConnecting to autopilot")
    conn_types = {"SC2A": "Surface Computer to Autopilot", "CC2A": "Companion Computer to Autopilot"}
    if conn_type == "SC2A":
        # create connection from surface computer to autopilot
        master, boot_time = helpers.create_master_SurfaceComputer2Autopilot()
    elif conn_type == "CC2A":
        # create connection from companion computer to autopilot
        master, boot_time = helpers.create_master_CompanionComputer2Autopilot()
        helpers.wait_conn(master)
    elif conn_type not in conn_types.keys():
        print(f"{conn_type} unsupported connection type. Available connection types:")
        for key in conn_types.keys():
            print(f" {key}: {conn_types[key]}")
        return(0)
    else:
        print(f"{conn_type} caused unexpected error.")
        return(0)

    # register the lambda function, to handle the ctrl+c signal
    signal.signal(signal.SIGINT, lambda signal, frame: exit_func(signal, frame, master))

    # clean up (disarm)
    print("Inital state")
    helpers.disarm(master)

    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'AUTO'
    print(f"Set flightmode to {flightmode}")
    helpers.change_flightmode(master, mode=flightmode)

    print("\n!!! Arming. Stay clear !!!")
    time_start = default_timer()
    countdown = 5
    while (default_timer() - time_start < countdown):
        print(round(countdown - (default_timer() - time_start)))
        time.sleep(1)
    # arm ardusub
    helpers.arm(master)

    ### BEGIN USER CODE ###
    
    test_set_servo_speed()
    
    ### END USER CODE ###
    
    # clean up (disarm)
    print("Program finished.")
    helpers.disarm(master)
    
    
def test_guided_speed():
    # (1) Set speed to 1 m/s
    print("\nSending MAV_CMD_GUIDED_CHANGE_SPEED (43000) with 1 m/s")
    helpers.send_cmd_guided_change_speed(master, 1)
    
    # (2) Wait 5s
    print("\nWait 5 seconds...")
    time_start = default_timer()
    countdown = 5
    while (default_timer() - time_start < countdown):
        print(round(countdown - (default_timer() - time_start)))
        time.sleep(1)
    
    # (3) Set speed back to 0 m/s
    print("\nSending MAV_CMD_GUIDED_CHANGE_SPEED (43000) with 0 m/s")
    helpers.send_cmd_guided_change_speed(master, 0)
    
def test_set_servo_speed():
        # (1) Set servo 1 to 1400 us
    print("\nSending MAV_CMD_DO_SET_SERVO (183) to servo 1 with 1400us")
    helpers.send_cmd_set_servo(master, 1, 1400)
    
    # (2) Wait 5s
    print("\nWait 5 seconds...")
    time_start = default_timer()
    countdown = 5
    while (default_timer() - time_start < countdown):
        print(round(countdown - (default_timer() - time_start)))
        time.sleep(1)    
        
def test_set_speed():
    # (1) Set speed to 1 m/s
    print("\nSending MAV_CMD_DO_CHANGE_SPEED (178) with 1 m/s")
    helpers.send_cmd_guided_change_speed(master, 1)
    
    # (2) Wait 5s
    print("\nWait 5 seconds...")
    time_start = default_timer()
    countdown = 5
    while (default_timer() - time_start < countdown):
        print(round(countdown - (default_timer() - time_start)))
        time.sleep(1)    