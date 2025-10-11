import os
import random
import math
import matplotlib
import statistics
import argparse
import time
import shutil
from operator import add

import customexceptions
import warehouse
import robot
import simulation

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", action="store_true", help="Whether or not to transmit UDP packets.")
    args = parser.parse_args()
    os.environ["ROBOTSIM_TRANSMIT"] = str(args.t)
    faulty = [0.0001, 0.001, 0.001, 0.001]
    perfect_scenario = [0, 0, 0, 0]

    sim = simulation.Simulation(1, "../data/whouse2.txt", 10, 3, "simple-interrupt",
                     perfect_scenario, True, 1000)

    sim.run_simulation(True,True)
    sim.print_priority_info()
