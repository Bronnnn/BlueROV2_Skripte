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

def standard_request_msg(master, mavlink_msg_id:int=29, param2:int=0):
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

def recv_match(master, timeout=1, mavpackettype = 'ATTITUDE'):
    # init timeout
    time_start = default_timer()
    time_passed = 0

    msg = None

    while time_passed<timeout:
        try:
            msg = master.recv_match(type=mavpackettype).to_dict()
            print("received")
            break
        except:
            time_passed = default_timer() - time_start
            print(f"retry: {(timeout-time_passed):.2f}s until timeout.")
        time.sleep(0.1)

    return msg


def create_master_CompanionComputer2Autopilot(addr:str="0.0.0.0:9000"):
    """
    Companion is already configured to allow script connections under the port 9000
    Note: The connection is done with 'udpout' and not 'udpin'.
    You can check in http:192.168.1.2:2770/mavproxy that the communication made for 9000
    uses a 'udp' (server) and not 'udpout' (client).
    """
    master = mavutil.mavlink_connection('udpout:'+addr)

    return master


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

def hello_world():
    print("hello world")
    print(f"current working dir: {os.getcwd()}")
