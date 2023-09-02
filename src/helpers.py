"""
Uses examples from https://www.ardusub.com/developers/pymavlink.html
Restructures the examples in functions
"""
import os

from pymavlink import mavutil
import sys
import time
import math
from timeit import default_timer
from pymavlink.quaternion import QuaternionBase

def arm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

def disarm(master):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    print("Waiting for the vehicle to disarm")
    master.motors_disarmed_wait()
    print('Disarmed!')

def manual_control(master, x, y, z, r):
    """
    Warning: Because of some legacy workaround, z will work between [0-1000]
    where 0 is full reverse, 500 is no output and 1000 is full throttle.
    x,y and r will be between [-1000 and 1000].
    """
    master.mav.manual_control_send(
        master.target_system,
        x=x,
        y=y,
        z=z,
        r=r,
        buttons=0
    )

def change_flightmode(master, mode='STABILIZE'):
    """
    available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    """
    # Check if mode is available
    if mode not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(master.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    # Set new mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

    while not master.wait_heartbeat().custom_mode == mode_id:
        master.set_mode(mode_id)

        # Wait for ACK command
        # Would be good to add mechanism to avoid endlessly blocking
        # if the autopilot sends a NACK or never receives the message
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        ack_msg = ack_msg.to_dict()

        # Print the ACK result !
        print(ack_msg)
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)

        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg['command'] == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            break

def set_target_depth(depth, master, boot_time):
    """
    Sets the target depth while in depth-hold mode.

    Uses https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT

    'depth' is technically an altitude, so set as negative meters below the surface
        -> set_target_depth(-1.5) # sets target to 1.5m below the water surface.
     """

    master.mav.set_position_target_global_int_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask=(  # ignore everything except z position
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                # DON'T mavutil.mavlink.POSITION_TARGET_TYPEMASK_FORCE_SET |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        ), lat_int=0, lon_int=0, alt=depth,  # (x, y WGS84 frame pos - not used), z [m]
        vx=0, vy=0, vz=0,  # velocities in NED frame [m/s] (not used)
        afx=0, afy=0, afz=0, yaw=0, yaw_rate=0
        # accelerations in NED frame [N], yaw, yaw_rate
        #  (all not supported yet, ignored in GCS Mavlink)
    )

def set_target_attitude(roll, pitch, yaw, master, boot_time):
    """
    Sets the target attitude while in depth-hold mode.
    Sets absolute angles (e.g. turn to +90°) and no relative angles (e.g. turn by +90°)
    Does not specify the rates.
    'roll', 'pitch', and 'yaw' are angles in degrees.
    """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
    )

def standard_request_msg(master, mavlink_msg_id, param2:int=0):
    """
    sends command MAV_CMD_REQUEST_MESSAGE (512)
    message id's and their description can be found on https://mavlink.io/en/messages/common.html
    """
    master.mav.command_long_send(
        target_system=master.target_system, #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component, #Target Components: Normally "0"
        command=512, #Command: MAV_CMD_REQUEST_MESSAGE
        confirmation=0, #Confirmation
        param1=mavlink_msg_id, #Param 1: check for msg id's and descriptions https://mavlink.io/en/messages/common.html
        param2=param2, #Param 2: Depends on message requested, see that messages definition for details
        param3=0, param4=0, param5=0, param6=0, param7=0) #Param 3 to 7: not used

def create_master_SurfaceComputer2Autopilot(timeout:int=5, addr:str="127.0.0.1:14551"):
    """
    If you’re running the GCS on the same machine as SITL then an appropriate output may already exist.
    Check this by calling output on the MAVProxy command prompt.

    If the GCS is connected to the same port as the python script of the surface computer, it will not be able to
    listen and therefore display the movement, while the script is communicationg with the simulated autopilot!!!

    Additional ports can be added by using: e.g. output add 192.168.14.82:14550
    Thats how you can use the MAVProxy (UDP) forwarding to connect additional GCS etc.
    """
    master = mavutil.mavlink_connection('udpin:'+addr)

    boot_time = time.time()
    # Make sure the connection is valid
    master.wait_heartbeat(timeout=timeout)

    return master, boot_time

def create_master_CompanionComputer2Autopilot(addr:str="0.0.0.0:14001"):
    """
    Companion is already configured to allow script connections under the port 9000
    Note: The connection is done with 'udpout' and not 'udpin'.
    You can check in http:192.168.1.2:2770/mavproxy that the communication made for 9000
    uses a 'udp' (server) and not 'udpout' (client).
    """
    master = mavutil.mavlink_connection('udpout:'+addr)

    boot_time = time.time()

    return master, boot_time
    
def recv_match(master, timeout=1, mavpackettype = 'ATTITUDE', verbose = 3):
    # init timeout
    time_start = default_timer()
    time_passed = 0

    msg = None

    while time_passed<timeout:
        try:
            msg = master.recv_match(type=mavpackettype).to_dict()
            if verbose>2: print("received")
            break
        except:
            time_passed = default_timer() - time_start
            if verbose>2: print(f"retry: {(timeout-time_passed):.2f}s until timeout.")
        time.sleep(0.1)

    return msg

def recv_parameter(master, timeout=1, param_id = 'SURFACE_DEPTH', verbose = 3):
    mavpackettype = 'PARAM_VALUE'

    # init timeout
    time_start = default_timer()
    time_passed = 0

    msg = None

    while time_passed<timeout:
        try:
            msg = master.recv_match(type=mavpackettype).to_dict()
            if msg['param_id'] != param_id: raise Exception(f"Expected message param id {param_id}. Received message with param id {msg['param_id']}.")
            if verbose>2: print("received")
            break
        except:
            time_passed = default_timer() - time_start
            if verbose>2: print(f"retry: {(timeout-time_passed):.2f}s until timeout.")
        time.sleep(0.1)

    return msg

def wait_conn(master):
    """
    Send a ping to start connection and wait for any reply.
    This function is necessary when using 'udpout',
    as described before, 'udpout' connects to 'udpin',
    and needs to send something to allow 'udpin' to start
    sending data.
    """
    msg = None
    while not msg:
        print("ping")
        master.mav.ping_send(
            int(time.time() * 1e6), # Unix time in microseconds
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)

    return 1

def check_capabilities(master, verbose=3):
    # get autopilot version and capabilities
    if verbose>0: print("Request 'AUTOPILOT_VERSION'")
    standard_request_msg(master, mavlink_msg_id=148)
    autopilot_version = recv_match(master, mavpackettype="AUTOPILOT_VERSION", verbose=0)
    if verbose>1: print(f"Autopilot version: {autopilot_version}")
    # check if autopilot supports commanding position and velocity targets in global scaled integers
    # Source: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = autopilot_version['capabilities'] & 256
    print(MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT)
    print(MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT==256)
    print(autopilot_version['capabilities'])
    print(autopilot_version['capabilities']==256)
    if verbose>0: print(f"MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT: {MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT == 256}")
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = autopilot_version['capabilities'] & 128
    if verbose>0: print(f"MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED: {MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED == 128}")

def init(master):
    # clean up (disarm)
    disarm(master)
    master.motors_disarmed_wait()

    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'ALT_HOLD'
    print(f"Set {flightmode} mode")
    change_flightmode(master, mode=flightmode)

    print("\n !!! Arming. Stay clear !!!")
    time_start = default_timer()
    countdown = 5
    while (default_timer() - time_start < countdown):
        print("\r"+str(round(countdown - (default_timer() - time_start))), end="")
        time.sleep(1)
    print(f"\n")
    # arm ardusub
    arm(master)
    master.motors_armed_wait()
    print(f"\n")

    return 1

def get_global_position_int(master, verbose=3):
    # get current depth
    if verbose>2: print("Request 'GLOBAL_POSITION_INT'")
    standard_request_msg(master, mavlink_msg_id=33)
    global_position_int = recv_match(master, mavpackettype="GLOBAL_POSITION_INT", verbose=verbose)

    return global_position_int

def update_position(master, target_depth_m:int, verbose = 3):
    position = {}
    global_position_int = get_global_position_int(master, verbose=verbose)
    position["depth_mm"] = global_position_int["alt"]
    position["depth_m"] = position["depth_mm"] / 1000
    position["depth_difference_abs_m"] = abs(position["depth_m"] - target_depth_m)
    position["heading"] = global_position_int["hdg"] / 100
    print(f"heading: {position['heading']}")
    #if position['heading'] < 0: position['heading'] = position['heading'] + 360

    return position

def print_position(position:dict, target_depth_m:int):
    print(f"current depth: {position['depth_m']:.2f}m")
    print(f"target depth: {target_depth_m:.2f}m")
    print(f"absolute depth difference: {position['depth_difference_abs_m']:.2f}m")
    print(f"heading: {position['heading']}°")

    return 1

def hold_depth(master, boot_time, target_depth_m, timeout_s, verbose = 3):
    """
    Links:
    Depth Hold Controller. https://github.com/ArduPilot/ardupilot/blob/fd32425d2495b681a9440f96f0be1c43142fbff5/ArduSub/control_althold.cpp
    Alt hold controller should be called at 100 Hz or more (does it mean target depth has to be set so often?)
    """
    print(f"=== Get to target depth: {target_depth_m}m ===")

    print(f"Target depth: {target_depth_m}m")
    set_surface_depth_parameter(master, -0.2)
    request_surface_depth_parameter(master)
    surface_depth_msg = recv_parameter(master, param_id='SURFACE_DEPTH', verbose=0)
    surface_depth_m = surface_depth_msg['param_value']/100
    adjusted_target_depth_m = target_depth_m + surface_depth_m
    print(f"Surface depth (parameter): {surface_depth_m}m")
    print(f"Adjusted target depth: {adjusted_target_depth_m}m")

    target_depth_reached_and_held = False
    while not target_depth_reached_and_held:
        # init timeout
        time_start = default_timer()
        time_passed = 0

        # get current depth
        if verbose > 0: print("\nUpdate position")
        position = update_position(master, target_depth_m, verbose=verbose)
        if verbose > 0: print_position(position, target_depth_m), time.sleep(2)

        # allowed difference between target depth and current depth
        max_depth_difference_m = 0.05
        timeout_passed = False
        target_depth_reached = False

        while not timeout_passed and not target_depth_reached:
            if verbose>2: print(f"Set target depth: {target_depth_m}m")
            set_target_depth(target_depth_m, master, boot_time)
            if verbose>1: print("\nUpdate position")
            position = update_position(master, adjusted_target_depth_m, verbose=verbose)
            if verbose>2: print_position(position, target_depth_m)
            time_passed = default_timer() - time_start
            if verbose>1: print(f"Target: {target_depth_m:.2f}m")
            if verbose>1: print(f"Adjusted Target (Target + Surface Depth Parameter): {adjusted_target_depth_m:.2f}m")
            if verbose>1: print(f"Reached: {(position['depth_m'])}m")
            if verbose > 1: print(f"Get to target depth: {(timeout_s - time_passed):.2f}s until timeout.")

            time.sleep(0.1)

            timeout_passed = time_passed > timeout_s
            target_depth_reached = position['depth_difference_abs_m'] < max_depth_difference_m

        time_start = default_timer()
        stabilization_time_s = 10
        print(f"\nSleep for {stabilization_time_s}s to stabilize.")
        while (default_timer() - time_start < stabilization_time_s):
            print("\r" + str(round(stabilization_time_s - (default_timer() - time_start))), end="")
            time.sleep(1)

        position = update_position(master, adjusted_target_depth_m, verbose=verbose)
        if verbose > 1: print(f"\n\nTarget: {target_depth_m}m")
        if verbose > 1: print(f"Adjusted Target (Target + Surface Depth Parameter): {adjusted_target_depth_m:.2f}m")
        if verbose > 1: print(f"Reached: {position['depth_m']}m")
        print(f"Check if the target depth has been held properly, otherwise retry setting target depth: ", end="")
        if position['depth_difference_abs_m'] < max_depth_difference_m:
            target_depth_reached_and_held = True
            print("Passed")
        else:
            target_depth_reached_and_held = False
            print("Failed")

def set_surface_depth_parameter(master, depth_m):
    # Set new parameter value
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b'SURFACE_DEPTH',
        depth_m*100,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

def request_surface_depth_parameter(master):
    # Request parameter
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        b'SURFACE_DEPTH', #parameter id
        -1
    )

def turn(master, relative_target_heading_deg, target_depth_m, timeout_s, verbose=3):
    """
    emulates joystick commands to turn. considers angle wrap arounds, when integrating.
    known issues: sometimes it does not handle angle wraparounds correctly and adds/subtracts 360°
    """
    print(f"\n=== Turn by {relative_target_heading_deg}° ===")
    # init timeout
    time_start = default_timer()
    time_passed = 0

    # get current depth
    print("Update position")
    position = update_position(master, target_depth_m, verbose=verbose)
    print(f"Heading: {position['heading']}°")

    # calculate desired heading and difference
    heading_old_deg = position['heading']
    num_target_full_turns = relative_target_heading_deg // 360
    fraction_turn_deg = relative_target_heading_deg - num_target_full_turns * 360
    absolute_target_heading = position['heading'] + fraction_turn_deg

    print(f"""
    Set target turn (relative to the current heading): {relative_target_heading_deg}°
    Current heading: {position['heading']}°
    Target turn (relativ to the current heading): {relative_target_heading_deg}°
    Requires the submarine to turn {num_target_full_turns} time(s) and by {fraction_turn_deg}°
    Results in a new absolute heading of {absolute_target_heading:.2f}°""", end="")

    # describes how much the submarines heading changed
    relative_heading_deg = 0
    num_full_turns_made = 0

    # allowed difference between relative target heading and current heading
    max_angle_difference_deg = 1
    timeout_passed = False
    relative_target_heading_reached = False

    while not timeout_passed and not relative_target_heading_reached:
        print("\rUpdate position")
        position = update_position(master, target_depth_m, verbose=verbose)
        # print_position(position, target_depth_m)
        heading_new_deg = position['heading']
        # check if the angle wrapped around (e.g. old_angle = 359° and new_angle = 1°).
        # assuming it turns slow enough, so that it doesnt turn by more
        # than 180° before a new heading reading is being compared.
        if heading_new_deg - heading_old_deg < -180: angle_wrap_right_turn = True
        elif heading_new_deg - heading_old_deg > 180: angle_wrap_left_turn = True
        else: angle_wrap_left_turn = angle_wrap_right_turn = False
        if angle_wrap_right_turn: relative_heading_deg += 360 - abs(heading_new_deg - heading_old_deg)
        elif angle_wrap_left_turn: relative_heading_deg -= 360 - abs(heading_new_deg - heading_old_deg)
        else: relative_heading_deg += heading_new_deg - heading_old_deg

        relative_heading_difference_abs_deg = abs(relative_target_heading_deg-relative_heading_deg)
        relative_heading_difference_deg = relative_target_heading_deg-relative_heading_deg

        if relative_heading_deg > 360+num_full_turns_made*360:
            num_full_turns_made +=1

        print(f"""
        Target (relative): {relative_target_heading_deg}°
        Reached (relative): {relative_heading_deg:.2f}°
        Target (absolute): {absolute_target_heading:.2f}°
        Reached (absolute): {heading_new_deg if num_full_turns_made==num_target_full_turns else 0}°
        Target (num complete turns): {num_target_full_turns}
        Reached (num complete turns): {num_full_turns_made}""", end="")

        # reduce power when close to target, to not overshoot (could have made a more sophisticated approach with pid
        # controller and such but this works aswell since our requirements are not that high)
        power = 200
        if relative_heading_difference_abs_deg < 10:
            power_tuning_factor = abs(1-relative_heading_deg/relative_target_heading_deg)
            if power_tuning_factor > 1: power_tuning_factor = 1
            power = int(power_tuning_factor*power)
            if power < 150: power = 150

        # turn
        if relative_heading_difference_deg > 0:
            print(f"{(power/1000)*100}% power rotating right", end="")
            manual_control(master, x=0, y=0, z=500, r=power)
        if relative_heading_difference_deg < 0:
            print(f"{(power/1000)*100}% power rotating left", end="")
            manual_control(master, x=0, y=0, z=500, r=-power)

        heading_old_deg = heading_new_deg

        # print the time left for reaching the target depth, before starting to rotate
        time_passed = default_timer() - time_start
        print(f"Make turn: {(timeout_s - time_passed):.2f}s until timeout.")
        print("\n")

        #time.sleep(0.5)
        timeout_passed = time_passed > timeout_s
        relative_target_heading_reached = relative_heading_difference_abs_deg < max_angle_difference_deg

    manual_control(master, x=0, y=0, z=500, r=0)
    print(f"\n")

def set_target_attitude(roll, pitch, yaw, master, boot_time):
    """ Sets the target attitude while in depth-hold mode.

        'roll', 'pitch', and 'yaw' are angles in degrees.

        """
    master.mav.set_attitude_target_send(
        int(1e3 * (time.time() - boot_time)),  # ms since boot
        master.target_system, master.target_component,
        # allow throttle to be controlled by depth_hold mode
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
        # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, 0  # roll rate, pitch rate, yaw rate, thrust
    )

def turn2(master, relative_target_heading_deg, target_depth_m, timeout_s, boot_time, verbose=3):
    """
    uses set attitude (with ignore throttle so that the depth hold controller can still work as in the example:
    https://www.ardusub.com/developers/pymavlink.html#set-target-depthattitude described), in combination with
    set target depth, sending the commands with more than 1 Hz.
    known issues: as soon as it starts turning with the set attitude command, the submarine no longer holds its depth.
    """
    print(f"\n=== Turn by {relative_target_heading_deg}° ===")
    # init timeout
    time_start = default_timer()
    time_passed = 0

    # get current depth
    print("Update position")
    position = update_position(master, target_depth_m, verbose=verbose)
    print(f"Heading: {position['heading']}°")

    # calculate desired heading and difference
    step_deg = 10
    heading_old_deg = position['heading']
    num_target_full_turns = relative_target_heading_deg // 360
    num_target_step_turns = relative_target_heading_deg // step_deg
    fraction_turn_deg = relative_target_heading_deg - num_target_step_turns * 10

    print(f"Set target turn (relative to the current heading): {relative_target_heading_deg}°")
    print(f"Current heading: {position['heading']}°")
    print(f"Target turn (relativ to the current heading): {relative_target_heading_deg}°")
    print(f"Requires the submarine to turn {num_target_full_turns} time(s) and {fraction_turn_deg}°")


    # allowed difference between relative target heading and current heading
    timeout_passed = False

    for target in range(num_target_step_turns):
        print("Update position")
        position = update_position(master, target_depth_m, verbose=verbose)
        print(f"Target Heading (relative): {relative_target_heading_deg}°")
        print(f"Reached Heading (relative): {position['heading']}")
        #manual_control(master, x=0, y=0, z=500, r=0)
        set_target_attitude(roll=0, pitch=0, yaw=position['heading']+target*step_deg, master=master, boot_time=boot_time)
        #for i in range(10):
        #    set_target_depth(target_depth_m, master, boot_time)
        #    time.sleep(0.1)
        #manual_control(master, x=0, y=0, z=500, r=0)
        print("\n")


    print(f"\n")

def hello_world():
    print("hello world")
    print(f"current working dir: {os.getcwd()}")

def send_cmd_guided_change_speed(master, speed:int=0):
    """
    sends command MAV_CMD_GUIDED_CHANGE_SPEED (43000) from ardupilotmega dialect
    
    https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_SPEED
    """
    master.mav.command_long_send(
        target_system=master.target_system,         #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component,   #Target Components: Normally "0"
        command=43000,                              #Command: MAV_CMD_GUIDED_CHANGE_SPEED
        confirmation=0,                             #Confirmation
        param1=1,                                   #Param 1: speed type (0 = Airspeed, = Groundspeed)
        param2=speed,                               #Param 2: speed target in [m/s]
        param3=0, param4=0, param5=0, param6=0, param7=0) #Param 3 to 7: not used
        
def send_cmd_guided_change_speed(master, speed:int=0, throttle:int=0):
    """
    sends command MAV_CMD_DO_CHANGE_SPEED (178)
    https://mavlink.io/en/messages/ardupilotmega.html#MAV_CMD_GUIDED_CHANGE_SPEED
    """
    master.mav.command_long_send(
        target_system=master.target_system,         #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component,   #Target Components: Normally "0"
        command=178,                                #Command: MAV_CMD_DO_CHANGE_SPEED
        confirmation=0,                             #Confirmation
        param1=1,                                   #Param 1: Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
        param2=speed,                               #Param 2: Speed (-1 indicates no change, -2 indicates return to default vehicle speed)
        param3=throttle,                            #Param 3: Throttle (-1 indicates no change, -2 indicates return to default vehicle throttle value)
        param4=0, param5=0, param6=0, param7=0)     #Param 3 to 7: not used
   
def send_cmd_set_servo(master, servo:int=0, pwm:int=0):
    """
    sends command MAV_CMD_DO_SET_SERVO (183)    
    https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO
    """
    master.mav.command_long_send(
        target_system=master.target_system,         #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component,   #Target Components: Normally "0"
        command=183,                                #Command: MAV_CMD_DO_SET_SERVO
        confirmation=0,                             #Confirmation
        param1=servo,                               #Param 1: Servo instance number 
        param2=pwm,                                 #Param 2: Pulse Width Modulation [us]
        param3=0, param4=0, param5=0, param6=0, param7=0) #Param 3 to 7: not used
   