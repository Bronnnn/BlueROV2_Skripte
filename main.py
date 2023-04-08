import sys

from src import helpers
from src import Teststand_Motorsequenz
from src import Teststand_Tauchsequenz_1
from src import Teststand_Tauchsequenz_2

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # helpers.hello_world()
    #Teststand_Motorsequenz.run(conn_type=sys.argv[1])
    #Teststand_Tauchsequenz.run(conn_type=sys.argv[1])
    #Teststand_Tauchsequenz_1.run(conn_type="SC2A")
    Teststand_Tauchsequenz_2.run(conn_type="SC2A")