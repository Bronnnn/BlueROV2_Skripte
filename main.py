# Test Commit von Laptop

import sys

from src import helpers
from src import Teststand_Motorsequenz
from src import Teststand_Tauchsequenz_1
from src import Teststand_Tauchsequenz_2
from src import Teststand_setTargetDepth_Experiment
from src import Teststand_Motorsequenz_Rampe
from src import ROV_Speedtest

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    available_sequences = ["Teststand_Motorsequenz", "Teststand_Tauchsequenz_1", "Teststand_Tauchsequenz_2",
                           "Teststand_setTargetDepth_Experiment", "Teststand_Motorsequenz_Rampe", "ROV_Speedtest"]
    available_connection_types = {"SC2A": "Surface Computer to Autopilot", "CC2A": "Companion Computer to Autopilot"}
    if len(sys.argv)<3:
        print(f"Requires 2 arguments: "
              f"\n1) sequence name (available sequences: {available_sequences})"
              f"\n2) connection type (avaialbe connection types: {available_connection_types})"
              f"\nExample: python main.py Teststand_Tauchsequenz_1 SC2A")

    sequence_name = sys.argv[1]
    conn_type = sys.argv[2]

    if sequence_name == "Teststand_Motorsequenz":
        Teststand_Motorsequenz.run(conn_type=conn_type)
    elif sequence_name == "Teststand_Tauchsequenz_1":
        Teststand_Tauchsequenz_1.run(conn_type=conn_type)
    elif sequence_name == "Teststand_Tauchsequenz_2":
        Teststand_Tauchsequenz_2.run(conn_type="SC2A")
    elif sequence_name == "Teststand_setTargetDepth_Experiment":
        Teststand_setTargetDepth_Experiment.run(conn_type=conn_type)
    elif sequence_name == "Teststand_Motorsequenz_Rampe":
        Teststand_Motorsequenz_Rampe.run(conn_type=conn_type)
    elif sequence_name == "ROV_Speedtest":
        ROV_Speedtest.run(conn_type=conn_type)
    else:
        print(f"{sequence_name} not a valid sequence name. "
              f"List of available sequence names: {available_sequences}")


