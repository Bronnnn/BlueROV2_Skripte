import sys

from src import helpers
from timeit import default_timer
import numpy as np
import signal
import time

def exit_func(signal, frame, master):
    print("Disarming before exiting")
    disarm(master)
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

    # Print chosen connection type
    print(f"Connection type: {conn_type}")
    
    # register the lambda function, to handle the ctrl+c signal
    signal.signal(signal.SIGINT, lambda signal, frame: exit_func(signal, frame, master))

    # clean up (disarm)
    print("Inital state")
    helpers.disarm(master)

    # available modes: ['STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'CIRCLE', 'SURFACE', 'POSHOLD', 'MANUAL']
    flightmode = 'MANUAL'
    print(f"Set flightmode to {flightmode}")
    helpers.change_flightmode(master, mode=flightmode)

    print("\n!!! Arming. Stay clear !!!")
    time_start = default_timer()
    countdown = 3
    while (default_timer() - time_start < countdown):
        print(round(countdown - (default_timer() - time_start)))
        time.sleep(1)
    # arm ardusub
    helpers.arm(master)

    ### BEGIN USER CODE ###
    
    #test_interval_msg(master)
    test_manual_control(master)
    
    ### END USER CODE ###
    
    # clean up (disarm)
    print("Program finished.")
    helpers.disarm(master)
    
    
def test_guided_speed(master):
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
    
def test_set_servo_speed(master):
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
        
def test_set_speed(master):
    # (1) Set speed to 1 m/s
    #print("\nSending MAV_CMD_DO_CHANGE_SPEED (178) with 1 m/s")
    #helpers.send_cmd_guided_change_speed(master, 1)
    
    time_start = default_timer()
    time_passed = 0
    timeout_s = 2
    #print("\nSending MAV_CMD_DO_CHANGE_SPEED (178) with 1 m/s")
    while time_passed < timeout_s:
        helpers.send_cmd_guided_change_speed(master, 1)
        time_passed = default_timer() - time_start
    
    # (2) Wait 5s
    print("\nWait 5 seconds...")
    time_start = default_timer()
    countdown = 5
    while (default_timer() - time_start < countdown):
        print(round(countdown - (default_timer() - time_start)))
        time.sleep(1)    
        
def test_manual_control(master):

    # Test to see if message is sent
    # (1) Set speed to 1 m/s
    #print("\nSending MAV_CMD_DO_CHANGE_SPEED (178) with 1 m/s")
    #helpers.send_cmd_guided_change_speed(master, 1)
    
    
    time.sleep(1.5)
    time_start = default_timer()
    time_passed = 0
    timeout_s = 5
    print("\nTesting z=450")
    while time_passed < timeout_s:
    
        # send x, y, z, r
        # x/y/r: [-1000 - 1000]
        # z: [0 - 1000]
        master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        r=0,
        z=450,
        buttons=0)
        
        time.sleep(0.05)
        time_passed = default_timer() - time_start
    
    # Set speed back to 0 in all directions
    master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        r=0,
        z=500,
        buttons=0)
    
    print("Wait 2s before next interval")
    #time.sleep(2)
    time_start = default_timer()
    time_passed = 0
    timeout_s = 5
    print("\nTesting x=50 (and z=500)")
    while time_passed < timeout_s:
    
        # send x, y, z, r with x/y/r: [-1000 - 1000] and z: [0 - 1000]
        master.mav.manual_control_send(
        master.target_system,
        x=50,
        y=0,
        r=0,
        z=500,
        buttons=0)
        
        time.sleep(0.05)
        time_passed = default_timer() - time_start
    
    print("Wait 2s before next interval")
    time.sleep(2)
    time_start = default_timer()
    time_passed = 0
    timeout_s = 5
    print("\nTesting r=50 (and z=500)")
    while time_passed < timeout_s:
        
        # send x, y, z, r with x/y/r: [-1000 - 1000] and z: [0 - 1000]
        master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        r=50,
        z=500,
        buttons=0)
        
        time.sleep(0.05)
        time_passed = default_timer() - time_start
    
    print("Wait 2s before next interval")
    time.sleep(2)
    time_start = default_timer()
    time_passed = 0
    timeout_s = 5
    print("\nTesting y=50(and z=500)")
    while time_passed < timeout_s:
        
        # send x, y, z, r with x/y/r: [-1000 - 1000] and z: [0 - 1000]
        master.mav.manual_control_send(
        master.target_system,

        x=0,
        y=50,
        r=0,
        z=500,
        buttons=0)
        
        time.sleep(0.05)
        time_passed = default_timer() - time_start
    
    # Get ESC Info
    master.mav.command_long_send(
        target_system=master.target_system,         #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component,   #Target Components: Normally "0"
        command=512,                                #Command: MAV_CMD_REQUEST_MESSAGE
        confirmation=0,                             #Confirmation
        param1=30,                                 #Param 1: Message ID (ESC_INFO #290)
        param2=0, param3=0, param4=0, param5=0, param6=0, param7=0) #Param 2 to 7: not used
    msg = helpers.recv_match(master, mavpackettype="ATTITUDE")
    print(msg)
    
    # Get ESC Info
    master.mav.command_long_send(
        target_system=master.target_system,         #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component,   #Target Components: Normally "0"
        command=512,                                #Command: MAV_CMD_REQUEST_MESSAGE
        confirmation=0,                             #Confirmation
        param1=290,                                 #Param 1: Message ID (ESC_INFO #290)
        param2=0, param3=0, param4=0, param5=0, param6=0, param7=0) #Param 2 to 7: not used
    msg = helpers.recv_match(master, mavpackettype="ESC_INFO")
    print(msg)
    
    # Get ESC Status
    master.mav.command_long_send(
        target_system=master.target_system,         #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component,   #Target Components: Normally "0"
        command=512,                                #Command: MAV_CMD_REQUEST_MESSAGE
        confirmation=0,                             #Confirmation
        param1=291,                                 #Param 1: Message ID (ESC_INFO #290)
        param2=0, param3=0, param4=0, param5=0, param6=0, param7=0) #Param 2 to 7: not used
    msg = helpers.recv_match(master = master, mavpackettype="ESC_STATUS")
    print(msg)

    # Get HIGH_LATENCY2
    master.mav.command_long_send(
        target_system=master.target_system,         #Target System: MAVLink system id of the vehicle (normally "1")
        target_component=master.target_component,   #Target Components: Normally "0"
        command=512,                                #Command: MAV_CMD_REQUEST_MESSAGE
        confirmation=0,                             #Confirmation
        param1=235,                                 #Param 1: Message ID (ESC_INFO #290)
        param2=0, param3=0, param4=0, param5=0, param6=0, param7=0) #Param 2 to 7: not used
    msg = helpers.recv_match(master, mavpackettype="HIGH_LATENCY2")
    print(msg)
        
def test_interval_msg(master):
    t = 0.0
    sp_x=sp_y=sp_r=0
    t1=t2=t3=t4=t5=0
    sp_z=500
    print("10s loop with different settings")
    while t < 10:
        print(f"time = {t}")
        # send manual control message
        master.mav.manual_control_send(
        master.target_system,
        x=sp_x,
        y=sp_y,
        r=sp_r,
        z=sp_z,
        buttons=0)
        
        # interval 1 (0-2s): dont do anything
        
        # interval 2 (2-4s): z = 450
        if t>2.0 and t1 == 0:
            z = 450
            print("z = 450")
            t1 = 1
        
        # interval 3 (4-6s): x = 50
        if t>4.0 and t2 == 0:
            z = 500
            x = 50
            print("x = 50")
            t2 = 1
        
        # interval 4 (6-8s): y = 50
        if t>6.0 and t3 == 0:
            x = 0
            y = 50
            print("y = 50")
            t3 = 1
        
        # interval 5 (8-10s): r = 50
        if t>8.0 and t4 == 0:
            y = 0
            r = 50
            print("r = 50")
            t4 = 1
            
        # increment time
        time.sleep(0.05)
        t += 0.05
        
        
    """
    master.mav.manual_control_send(
        master.target_system,
        x=0,
        y=0,
        r=0,
        z=500,
        buttons=0)
    """