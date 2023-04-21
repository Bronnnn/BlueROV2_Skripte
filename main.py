import sys

from src import helpers
from src import Teststand_Motorsequenz
from src import Teststand_Tauchsequenz_1
from src import Teststand_Tauchsequenz_2

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    available_sequences = ["Teststand_Motorsequenz", "Teststand_Tauchsequenz_1", "Teststand_Tauchsequenz_2"]
    sequence_name = sys.argv[1]
    conn_type = sys.argv[2]

    if sequence_name == "Teststand_Motorsequenz":
        Teststand_Motorsequenz.run(conn_type=conn_type)
    elif sequence_name == "Teststand_Tauchsequenz_1":
        Teststand_Tauchsequenz_1.run(conn_type=conn_type)
    elif sequence_name == "Teststand_Tauchsequenz_2":
        Teststand_Tauchsequenz_2.run(conn_type="SC2A")
    else:
        print(f"{sequence_name} not a valid sequence name. "
              f"List of available sequence names: {available_sequences}")


