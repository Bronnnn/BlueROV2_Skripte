from src import helpers
from timeit import default_timer
import time

def run(conn_type:str="SC2A"):
    """
    emulates joystick commands
    """
    print("Teststand_Motorsequenz")

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
    time_start = default_timer()
    time_passed = 0
    timeout_s = 5
    print("medium speed down")
    while time_passed < timeout_s:
        helpers.manual_control(master, x=0, y=0, z=250, r=0)
        time_passed = default_timer() - time_start

    # init timer
    time_start = default_timer()
    time_passed = 0
    timeout_s = 5
    print("medium speed rotating right")
    while time_passed < timeout_s:
        helpers.manual_control(master, x=0, y=0, z=500, r=500)
        time_passed = default_timer() - time_start

    # init timer
    time_start = default_timer()
    time_passed = 0
    timeout_s = 5
    print("medium speed forward")
    while time_passed < timeout_s:
        helpers.manual_control(master, x=500, y=0, z=500, r=0)
        time_passed = default_timer() - time_start

    # init timer
    time_start = default_timer()
    time_passed = 0
    timeout_s = 5
    print("medium speed sideways right")
    while time_passed < timeout_s:
        helpers.manual_control(master, x=0, y=500, z=500, r=0)
        time_passed = default_timer() - time_start