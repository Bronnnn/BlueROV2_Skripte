from src import helpers
from timeit import default_timer
import time

def run(conn_type:str="SC2A"):
    """
    emulates joystick commands to turn
    """
    print("Teststand_Tauchsequenz")

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

    # init submarine
    helpers.init(master)
    helpers.check_capabilities(master)

    # set parameters
    target_depth_m = -2
    timeout_s = 60

    # update depth
    position = helpers.update_position(master, target_depth_m)
    helpers.print_position(position, target_depth_m)

    # dive to depth
    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'ALT_HOLD'
    print(f"Set {flightmode} mode")
    helpers.change_flightmode(master, mode=flightmode)
    helpers.hold_depth(master, boot_time, target_depth_m, timeout_s, verbose=2)



    # turn
    helpers.turn(master, 360, target_depth_m, timeout_s, verbose=0)

    time_wait_s = 5
    print(f"sleep for {time_wait_s}s to stabilize")
    print(time.sleep(time_wait_s))

    # dive to depth
    target_depth_m = 0
    helpers.hold_depth(master, boot_time, target_depth_m, timeout_s)

    time_wait_s = 5
    print(f"sleep for {time_wait_s}s to stabilize")
    print(time.sleep(time_wait_s))


