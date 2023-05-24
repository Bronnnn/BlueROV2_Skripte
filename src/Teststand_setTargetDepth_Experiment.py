from src import helpers
from timeit import default_timer
import time

def run(conn_type:str="SC2A"):
    """
    uses turn(): emulates joystick commands
    """
    print("Teststand_Tauchsequenz 1")

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
    helpers.check_capabilities(master, verbose=1)

    # set parameters
    for step in range(10):
        depth_m = step
        print(f"Set surface depth to {step}m")
        helpers.set_surface_depth_parameter(master = master, depth_m = depth_m)
        msg = helpers.recv_parameter(master = master, timeout = 1, param_id = "SURFACE_DEPTH", verbose = 3)
        print(f"Received msg: {msg}")
        print("Sleep 2 seconds")
        time.sleep(2)