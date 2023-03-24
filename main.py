from src import helpers
from src import Teststand_Motorsequenz
from src import BlueSim_Motorsequenz

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # helpers.hello_world()
    # Teststand_Motorsequenz.run()
    BlueSim_Motorsequenz.run(conn_type="SC2A")