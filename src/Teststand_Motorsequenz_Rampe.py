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
    
def run(conn_type:str="CC2A"):
    """
    emulates joystick commands and slowly ramps them up

    *DO NOT* run thrusters for longer than 30 seconds in air or you will wear out the plastic bearings.
    Source: https://bluerobotics.com/learn/bluerov2-software-setup/

    Here: 3 * 2s per direction and 4 different directions = 24s (plus sleep time)
    """
    print("Teststand_Motorsequenz_Rampe")
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

    print(f"Connection type: {conn_type}")
    # register the lambda function, to handle the ctrl+c signal
    signal.signal(signal.SIGINT, lambda signal, frame: exit_func(signal, frame, master))

    # clean up (disarm)
    print("Inital state")
    helpers.disarm(master)

    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'MANUAL'
    print(f"Set {flightmode} mode")
    helpers.change_flightmode(master, mode=flightmode)

    print("\n!!! Arming. Stay clear !!!")
    time_start = default_timer()
    countdown = 5
    while (default_timer() - time_start < countdown):
        print(round(countdown - (default_timer() - time_start)))
        time.sleep(1)
    # arm ardusub
    helpers.arm(master)

    for speed in np.arange(start=500, stop=374, step=-25):
        # init timer
        time_start = default_timer()
        time_passed = 0
        run_s = 2
        print(f"z = {speed} (down; neutral 500) for {run_s}s")
        while time_passed < run_s:
            helpers.manual_control(master, x=0, y=0, z=speed, r=0)
            time_passed = default_timer() - time_start

        # init timer
        time_start = default_timer()
        time_passed = 0
        sleep_s = 4
        print(f"sleeping {sleep_s}s")
        while time_passed < sleep_s:
            helpers.manual_control(master, x=0, y=0, z=500, r=0)
            time_passed = default_timer() - time_start

    for speed in np.arange(start=0, stop=101, step=25):
        # init timer
        time_start = default_timer()
        time_passed = 0
        run_s = 2
        print(f"r = {speed} (rotating right) for {run_s}s")
        while time_passed < run_s:
            helpers.manual_control(master, x=0, y=0, z=500, r=speed)
            time_passed = default_timer() - time_start

        # init timer
        time_start = default_timer()
        time_passed = 0
        sleep_s = 4
        print(f"sleeping {sleep_s}s")
        while time_passed < sleep_s:
            helpers.manual_control(master, x=0, y=0, z=500, r=0)
            time_passed = default_timer() - time_start

    for speed in np.arange(start=0, stop=101, step=25):
        # init timer
        time_start = default_timer()
        time_passed = 0
        run_s = 2
        print(f"x = {speed} (forward) for {run_s}s")
        while time_passed < run_s:
            helpers.manual_control(master, x=speed, y=0, z=500, r=0)
            time_passed = default_timer() - time_start

        # init timer
        time_start = default_timer()
        time_passed = 0
        sleep_s = 4
        print(f"sleeping {sleep_s}s")
        while time_passed < sleep_s:
            helpers.manual_control(master, x=0, y=0, z=500, r=0)
            time_passed = default_timer() - time_start

    for speed in np.arange(start=0, stop=101, step=25):
        # init timer
        time_start = default_timer()
        time_passed = 0
        run_s = 2
        print(f"y = {speed} (sideways, right) for {run_s}s")
        while time_passed < run_s:
            helpers.manual_control(master, x=0, y=speed, z=500, r=0)
            time_passed = default_timer() - time_start

        # init timer
        time_start = default_timer()
        time_passed = 0
        sleep_s = 4
        print(f"sleeping {sleep_s}s")
        while time_passed < sleep_s:
            helpers.manual_control(master, x=0, y=0, z=500, r=0)
            time_passed = default_timer() - time_start

    # clean up (disarm)
    print("Inital state")
    helpers.disarm(master)