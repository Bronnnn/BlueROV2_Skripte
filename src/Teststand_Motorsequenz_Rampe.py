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
    """
    print("Teststand_Motorsequenz_Rampe")

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

    # register exit funtion to handle the ctrl+c signal
    signal.signal(signal.SIGINT, exit_func)

    # clean up (disarm)
    print("Inital state")
    helpers.disarm(master)

    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'STABILIZE'
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

    # init timer
    for speed in np.arange(start = 500, stop = 450, step = 10):
        time_start = default_timer()
        time_passed = 0
        timeout_s = 2
        print(f"z = {speed} (down; neutral 500)")
        print(f"{time_passed / timeout_s}s")
        while time_passed < timeout_s:
            helpers.manual_control(master, x=0, y=0, z=speed, r=0)
            time_passed = default_timer() - time_start
        sleep_time_s = 4
        print(f"sleeping {sleep_time_s}s")
        time.sleep(sleep_time_s)

    # init timer
    for speed in np.arange(start=0, stop=50, step=10):
        time_start = default_timer()
        time_passed = 0
        timeout_s = 2
        print(f"r = {speed} (rotating right)")
        print(f"{time_passed / timeout_s}s")
        while time_passed < timeout_s:
            helpers.manual_control(master, x=0, y=0, z=500, r=speed)
            time_passed = default_timer() - time_start
        sleep_time_s = 4
        print(f"sleeping {sleep_time_s}s")
        time.sleep(sleep_time_s)

    # init timer
    for speed in np.arange(start=0, stop=50, step=10):
        time_start = default_timer()
        time_passed = 0
        timeout_s = 2
        print(f"x = {speed} (forward)")
        print(f"{time_passed / timeout_s}s")
        while time_passed < timeout_s:
            helpers.manual_control(master, x=speed, y=0, z=500, r=0)
            time_passed = default_timer() - time_start
        sleep_time_s = 4
        print(f"sleeping {sleep_time_s}s")
        time.sleep(sleep_time_s)

    # init timer
    for speed in np.arange(start=0, stop=50, step=10):
        time_start = default_timer()
        time_passed = 0
        timeout_s = 2
        print(f"y = {speed} (sideways, right)")
        print(f"{time_passed/timeout_s}s")
        while time_passed < timeout_s:
            helpers.manual_control(master, x=0, y=speed, z=500, r=0)
            time_passed = default_timer() - time_start
        sleep_time_s = 4
        print(f"sleeping {sleep_time_s}s")
        time.sleep(sleep_time_s)