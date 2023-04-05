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
    timeout_s = 15

    # update depth
    position = helpers.update_position(master, target_depth_m)
    helpers.print_position(position, target_depth_m)

    # dive to depth
    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'ALT_HOLD'
    print(f"Set {flightmode} mode")
    helpers.change_flightmode(master, mode=flightmode)
    hold_depth(master, boot_time, target_depth_m, timeout_s)

    time_wait_s = 5
    print(f"sleep for {time_wait_s}s to stabilize")
    print(time.sleep(time_wait_s))

    # turn
    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'STABILIZE'
    print(f"Set {flightmode} mode")
    helpers.change_flightmode(master, mode=flightmode)
    turn(master, 360, target_depth_m, timeout_s)

    time_wait_s = 5
    print(f"sleep for {time_wait_s}s to stabilize")
    print(time.sleep(time_wait_s))

    # dive to depth
    target_depth_m = 0
    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'ALT_HOLD'
    print(f"Set {flightmode} mode")
    helpers.change_flightmode(master, mode=flightmode)
    hold_depth(master, boot_time, target_depth_m, timeout_s)

    time_wait_s = 5
    print(f"sleep for {time_wait_s}s to stabilize")
    print(time.sleep(time_wait_s))

def hold_depth(master, boot_time, target_depth_m, timeout_s):
    """
    Links:
    Depth Hold Controller. https://github.com/ArduPilot/ardupilot/blob/fd32425d2495b681a9440f96f0be1c43142fbff5/ArduSub/control_althold.cpp
    Alt hold controller should be called at 100 Hz or more (does it mean target depth has to be set so often?)
    """

    # init timeout
    time_start = default_timer()
    time_passed = 0

    # get current depth
    print("Update position")
    position = helpers.update_position(master, target_depth_m)
    helpers.print_position(position, target_depth_m)

    # allowed difference between target depth and current depth
    max_depth_difference_m = 0.1
    timeout_passed = False
    target_depth_reached = False

    while not timeout_passed and not target_depth_reached:
        print(f"Set target depth: {target_depth_m}m")
        helpers.set_target_depth(target_depth_m, master, boot_time)

        print("Update position")
        position = helpers.update_position(master, target_depth_m)
        helpers.print_position(position, target_depth_m)

        # print the time left for reaching the target depth, before starting to rotate
        time_passed = default_timer() - time_start
        print(f"Get to target depth: {(timeout_s - time_passed):.2f}s until timeout.")
        print("\n")
        time.sleep(0.1)

        timeout_passed = time_passed > timeout_s
        target_depth_reached = position['depth_difference_abs_m'] < max_depth_difference_m

    print(f"\n")

def turn(master, relative_target_heading_deg, target_depth_m, timeout_s):
    # init timeout
    time_start = default_timer()
    time_passed = 0

    # get current depth
    print("Update position")
    position = helpers.update_position(master, target_depth_m)
    helpers.print_position(position, target_depth_m)
    heading_old_deg = position['heading']
    relative_heading_deg = 0

    # allowed difference between relative target heading and current heading
    max_angle_difference_deg = 1
    timeout_passed = False
    relative_target_heading_reached = False

    while not timeout_passed and not relative_target_heading_reached:
        print(f"Set target turn (relative to the current heading): {relative_target_heading_deg}deg")
        print("Update position")
        position = helpers.update_position(master, target_depth_m)
        helpers.print_position(position, target_depth_m)
        heading_new_deg = position['heading']
        relative_heading_deg += heading_new_deg - heading_old_deg
        relative_heading_difference_abs_deg = abs(relative_target_heading_deg-relative_heading_deg)
        print(f"turn difference abs: {relative_heading_difference_abs_deg:.2f}°")
        # to not overshoot too much
        coefficient = relative_heading_difference_abs_deg/abs(relative_target_heading_deg)
        print(f"coefficient: {coefficient}")

        # turn
        if relative_target_heading_deg > 0:
            print("medium speed rotating right")
            helpers.manual_control(master, x=0, y=0, z=500, r=int(500*coefficient))
        if relative_target_heading_deg < 0:
            print("medium speed rotating left")
            helpers.manual_control(master, x=0, y=0, z=500, r=-500)

        heading_old_deg = heading_new_deg

        # print the time left for reaching the target depth, before starting to rotate
        time_passed = default_timer() - time_start
        print(f"Make turn: {(timeout_s - time_passed):.2f}s until timeout.")
        print("\n")
        time.sleep(0.1)

        timeout_passed = time_passed > timeout_s
        relative_target_heading_reached = relative_heading_difference_abs_deg < max_angle_difference_deg

    print(f"\n")
