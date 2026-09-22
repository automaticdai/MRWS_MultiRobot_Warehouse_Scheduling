from mrws.exceptions import SimulationError
from mrws.engine.warehouse import Warehouse
import os
import argparse
import time
import statistics
import shutil
import math
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy
import numpy as np

from operator import add

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
DEFAULT_TRANSMIT_DELAY_S = 0.2
PRIORITY_LEVELS = 5

# Warehouse the batch experiments run on. This used to be spelled out at every
# call site as "whouse2.txt", which was renamed to whouse7x11.txt without the
# call sites being updated, so every batch experiment raised FileNotFoundError.
DEFAULT_EXPERIMENT_WAREHOUSE = os.path.join(DATA_DIR, "whouse7x11.txt")
DEFAULT_ROBOT_INVENTORY = 3


def count_shelves(warehouse_file):
    """Shelves in a warehouse file, which is how many items it can hold.

    Warehouse.parse_warehouse_file rejects a mismatch between this and the item
    count, so deriving it beats hard-coding a number next to a filename.
    """
    with open(warehouse_file) as handle:
        return handle.read().count("S")


class Simulation:
    def __init__(
        self,
        num_sims: int,
        whouse: str,
        num_items: int,
        inv_size: int,
        schedule_mode: str,
        fault_rates,
        fault_mode,
        step_limit,
        side_len=None,
    ):
        self._side_len = side_len
        self._num_sims = num_sims
        self.warehouse_file = whouse
        self._num_items = num_items
        self._inv_size = inv_size
        self._schedule_mode = schedule_mode
        self._fault_rates = fault_rates
        self._fault_mode = fault_mode
        self._step_limit = step_limit

        self.num_robots = None

        self.step_amounts = []
        self.step_times = []
        self.error_strings = []
        self.ga_attempts = [0, 0, 0, 0, 0]

        self.order_prio = []
        self.order_prio_sorted_completion_time_lists = [[] for _ in range(PRIORITY_LEVELS)]

        self.order_completion_steps_by_num_robots = {}

    def _record_order_completion_times(self, simu):
        orders_to_amount_robots = simu.get_scheduler().get_order_to_amount_of_robots_assigned()
        order_start_times = simu.get_order_manager().get_order_start_work_times()
        order_finish_times = simu.get_order_manager().get_order_finish_work_times()

        for order_name, amount_robots in orders_to_amount_robots.items():
            order_total_time = order_finish_times[order_name] - order_start_times[order_name]
            self.order_completion_steps_by_num_robots.setdefault(amount_robots, []).append(order_total_time)

    def _record_priority_completion_times(self):
        for prio, completion_time in self.order_prio:
            prio_index = prio - 1
            if 0 <= prio_index < PRIORITY_LEVELS:
                self.order_prio_sorted_completion_time_lists[prio_index].append(completion_time)

    def run_simulation(self, reraise_error, slow_for_transmit):
        for sim_num in range(self._num_sims):
            print(f"Starting sim {sim_num}")
            sim_step_times = []
            try:
                simu = Warehouse(
                    self.warehouse_file,
                    self._num_items,
                    self._inv_size,
                    self._schedule_mode,
                    self._fault_rates,
                    self._fault_mode,
                    self._step_limit,
                )
                keep_step = True
                while keep_step:
                    if slow_for_transmit:
                        time.sleep(DEFAULT_TRANSMIT_DELAY_S)
                    before_step_time = time.perf_counter_ns()
                    keep_step = not simu.step()
                    elapsed_time_between = time.perf_counter_ns() - before_step_time
                    sim_step_times.append(elapsed_time_between)

            except SimulationError as err:
                if reraise_error:
                    raise err
                self.error_strings.append(err)
                continue
            except Exception as err:
                if reraise_error:
                    raise err
                continue

            self.step_times.extend(sim_step_times)
            self.step_amounts.append(simu.get_total_steps())
            self.order_prio.extend(simu.get_order_manager().return_mapping_prio_to_completion_times())
            self.ga_attempts = list(map(add, simu.get_scheduler().get_ga_attempts(), self.ga_attempts))
            self._record_order_completion_times(simu)
            self.num_robots = simu.get_number_of_robots()

        self._record_priority_completion_times()

    def print_priority_info(self):
        plt.figure(figsize=(8, 8))

        for j in range(PRIORITY_LEVELS):
            if self.order_prio_sorted_completion_time_lists[j]:
                sel = self.order_prio_sorted_completion_time_lists[j]
                print(f"Average amount of steps for prio {j + 1} completion: {statistics.mean(sel)}")
        plt.boxplot(self.order_prio_sorted_completion_time_lists)

        ax = plt.gca()
        plt.title(f"Order prioritisation statistics for scheduling algorithm {self._schedule_mode}")
        ax.set_xlabel("Order priority")
        ax.set_ylabel("Amount of steps to complete order after introduction")

        plt.show()

    def print_num_robots_info(self):
        for amount_bots, completion_times in self.order_completion_steps_by_num_robots.items():
            print(f"Average amount of time taken for an order with {amount_bots} bots: {statistics.mean(completion_times)}")

    def print_error_info(self):
        for err in self.error_strings:
            print(err)
        print(f"{len(self.error_strings)} simulations faulted critically.")
        return self.error_strings

    def print_steps_taken(self):
        print(f"Simulation {self._schedule_mode} took an average of {statistics.mean(self.step_amounts)} steps")
        return self.step_amounts

    def print_step_time_info(self):
        max_step_ms = max(self.step_times) / 10**6
        min_step_ms = min(self.step_times) / 10**6
        mean_step_ms = statistics.mean(self.step_times) / 10**6
        print(f"Max step time {max_step_ms}ms, Min step time {min_step_ms}ms")
        print(f"Mean step time {mean_step_ms}ms")

        return [self.num_robots, self._side_len, mean_step_ms]


def gen_nxn_warehouse(robot_num, side_len, num_items=12, output_dir=None):
    if output_dir is None:
        output_dir = DATA_DIR
    filename = os.path.join(output_dir, f"wt{side_len}x{side_len}r{robot_num}.txt")

    # Build grid as 2D list of characters, all empty initially
    grid = [['X'] * side_len for _ in range(side_len)]

    # Place robots in top rows, filling row by row (cols 1..side_len-2)
    robots_placed = 0
    usable_cols = side_len - 2  # leave first and last column free
    robot_rows_needed = math.ceil(robot_num / usable_cols)
    for r in range(robot_rows_needed):
        for c in range(1, side_len - 1):
            if robots_placed >= robot_num:
                break
            grid[r][c] = 'R'
            robots_placed += 1

    if robots_placed < robot_num:
        raise Exception(f"Not enough space for {robot_num} robots in a {side_len}x{side_len} warehouse")

    # Place goals in bottom rows, filling row by row from the bottom (cols 1..side_len-2)
    goals_placed = 0
    goals_needed = max(robot_num, 1)
    goal_rows_needed = math.ceil(goals_needed / usable_cols)
    for r in range(goal_rows_needed):
        row_idx = side_len - 1 - r
        for c in range(1, side_len - 1):
            if goals_placed >= goals_needed:
                break
            grid[row_idx][c] = 'G'
            goals_placed += 1

    # Interior region for shelves: between robot rows and goal rows
    shelf_start_row = robot_rows_needed + 1  # +1 for corridor
    shelf_end_row = side_len - 1 - goal_rows_needed - 1  # -1 for corridor

    # Place shelves in alternating columns (shelf column, corridor column)
    shelves_placed = 0
    for c in range(2, side_len - 2):
        if shelves_placed >= num_items:
            break
        # Alternating: even offset columns get shelves, odd are corridors
        if (c - 2) % 2 != 0:
            continue
        for r in range(shelf_start_row, shelf_end_row + 1):
            if shelves_placed >= num_items:
                break
            grid[r][c] = 'S'
            shelves_placed += 1

    if shelves_placed < num_items:
        raise Exception(f"Not enough space for {num_items} shelves in a {side_len}x{side_len} warehouse")

    with open(filename, "w") as f:
        for r in range(side_len):
            line = ''.join(grid[r])
            if r < side_len - 1:
                line += "\n"
            f.write(line)

    return filename

def run_simulation_performance_test(scheduling_mode: str, robots_max: int, size_max: int, step_limit: int):
    tmp_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tmp")
    if os.path.isdir(tmp_dir):
        shutil.rmtree(tmp_dir)
    os.makedirs(tmp_dir)

    results = []
    by_sim_size = []
    ctr = 0
    for side_len in range(7, size_max, 2):
        by_sim_size.append([])
        for robot_count in range(1, robots_max + 1):
            print(f"starting {robot_count} {side_len}")
            file_name = gen_nxn_warehouse(robot_count, side_len, 12, output_dir=tmp_dir)
            sim_1 = Simulation(10, file_name, 12, 3, scheduling_mode,
                     [0, 0, 0, 0], True, step_limit, side_len)
            sim_1.run_simulation(False, False)
            sim_result = sim_1.print_step_time_info()
            results.append(sim_result)
            by_sim_size[ctr].append(sim_result)
        ctr += 1
    print(results)

    res = numpy.array(results)
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    x = res[:, 0]
    y = res[:, 1]
    z = res[:, 2]
    plt.title("Simulation performance for scheduling algorithm %s" % scheduling_mode)


    for robot_line in by_sim_size:
        mini_res = numpy.array(robot_line)
        ax.plot(mini_res[:, 0], mini_res[:, 1], mini_res[:, 2], color="grey")
    ax.scatter(x, y, z, c=z, cmap=matplotlib.colormaps.get_cmap("inferno"))



    ax.set_xlabel('Amount of Robots')
    ax.set_ylabel('Warehouse side length')
    ax.set_zlabel('Average step time (ms)')

    ax.xaxis.set_major_locator(ticker.IndexLocator(1,0))
    ax.yaxis.set_major_locator(ticker.IndexLocator(2, 0))

    plt.show()


def run_completion_time_test(fault_rates, num_sims=500, whouse=DEFAULT_EXPERIMENT_WAREHOUSE):
    num_items = count_shelves(whouse)

    sim = Simulation(num_sims, whouse, num_items, DEFAULT_ROBOT_INVENTORY, "simple",
                     fault_rates, True, 1000)

    sim_1 = Simulation(num_sims, whouse, num_items, DEFAULT_ROBOT_INVENTORY, "simple-interrupt",
                       fault_rates, True, 1000)

    sim_2 = Simulation(num_sims, whouse, num_items, DEFAULT_ROBOT_INVENTORY, "multi-robot",
                       fault_rates, True, 1000)

    sim_3 = Simulation(num_sims, whouse, num_items, DEFAULT_ROBOT_INVENTORY, "multi-robot-genetic",
                       fault_rates, True, 1000)

    sim.run_simulation(False, False)
    sim_1.run_simulation(False, False)
    sim_2.run_simulation(False, False)
    sim_3.run_simulation(False, False)

    steps = [sim.print_steps_taken(), sim_1.print_steps_taken(), sim_2.print_steps_taken(), sim_3.print_steps_taken()]

    fig = plt.figure(figsize=(8, 8))
    plt.boxplot(steps)
    ax = plt.gca()
    plt.title("Amount of time steps to complete simulation for each scheduling mode")
    plt.xticks([1, 2, 3, 4], ['simple', 'simple-interrupt', 'multi-robot', 'multi-robot-genetic'])
    ax.set_xlabel("Scheduling mode")
    ax.set_ylabel("Amount of steps")
    plt.show()

def _count_error_types(errors):
    error_types = [len(errors), 0, 0, 0]
    for error_message in errors:
        message = str(error_message)
        if "collided" in message:
            error_types[1] += 1
        if "limit" in message:
            error_types[3] += 1
        if "empty" in message or "violated" in message:
            error_types[2] += 1
    return error_types


def run_fault_test(scheduling_mode, num_sims=250, whouse=DEFAULT_EXPERIMENT_WAREHOUSE):
    faulty = [0.0001, 0.001, 0.001, 0.001]
    num_items = count_shelves(whouse)

    sim = Simulation(num_sims, whouse, num_items, DEFAULT_ROBOT_INVENTORY, scheduling_mode,
                     faulty, True, 1500)

    sim_1 = Simulation(num_sims, whouse, num_items, DEFAULT_ROBOT_INVENTORY, scheduling_mode,
                       faulty, False, 1500)



    sim.run_simulation(False, False)
    sim_1.run_simulation(False, False)


    all_errors_1 = sim.print_error_info()
    all_errors_2 = sim_1.print_error_info()

    error_types_1 = _count_error_types(all_errors_1)
    error_types_2 = _count_error_types(all_errors_2)

    types = ("Total Critically Faulted Simulations", "Collisions", "Scheduling Violation", "Overruns")

    x = np.arange(len(types))  # the label locations
    width = 0.25  # the width of the bars
    multiplier = 0.5

    groups = {
        "Fault tolerant strategy enabled": error_types_1,
        "Fault tolerant strategy disabled": error_types_2
    }

    colors = [
        (0.184, 0.404, 0.692, 1.0),
        (0.749, 0.172, 0.137, 1.0),
    ]

    fig = plt.figure(figsize=(8,8))
    ax = fig.gca()

    color_ctr = 0
    for attribute, measurements in groups.items():

        offset = width * multiplier
        rects = ax.bar(x + offset, measurements, width, label=attribute, color=colors[color_ctr])
        ax.bar_label(rects, padding=3)
        multiplier += 1
        color_ctr += 1

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('Amount')
    ax.set_title('Fault tolerant strategy for scheduling mode %s, \ntotal amount of simulations for each mode n=%s,\nsimulation cutoff 1500 steps'
                 % (scheduling_mode, num_sims))
    ax.set_xticks(x + width, types)
    ax.legend(loc='upper left', ncols=3)
    ax.set_ylim(0, error_types_2[0] + 50)
    plt.show()




def run_1000_robot_test():
    os.environ["ROBOTSIM_TRANSMIT"] = "False"
    fname = gen_nxn_warehouse(1000, 200, 1000)
    sim = Simulation(1, fname, 1000, 3, "multi-robot",
                     [0, 0, 0, 0], True, 5000)
    t = time.time()
    sim.run_simulation(True, False)
    print("1000-robot sim completed in %.1fs" % (time.time() - t))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--transmit", action="store_true", help="Whether or not to transmit UDP packets.")
    parser.add_argument(
        "-n",
        "--num-sims",
        type=int,
        default=1000,
        help="Number of simulation cycles to run (default: 1000).",
    )
    parser.add_argument(
        "-g",
        "--gui",
        action="store_true",
        help="Launch PyQt6 debug GUI (ignores -n).",
    )
    parser.add_argument(
        "-m",
        "--mode",
        type=str,
        default="simple-interrupt",
        choices=["simple", "simple-interrupt", "multi-robot", "multi-robot-genetic"],
        help="Scheduling mode.",
    )
    parser.add_argument(
        "-w",
        "--warehouse",
        type=str,
        default=os.path.join(DATA_DIR, "whouse.txt"),
        help="Path to warehouse file (default: data/whouse.txt).",
    )
    parser.add_argument(
        "-i",
        "--items",
        type=int,
        default=None,
        help="Number of items/shelves (default: auto-detect from warehouse file).",
    )
    args = parser.parse_args()
    os.environ["ROBOTSIM_TRANSMIT"] = str(args.transmit)

    # Auto-detect shelf count from the warehouse file if not specified
    if args.items is None:
        with open(args.warehouse) as f:
            args.items = f.read().count('S')

    perfect_scenario = [0, 0, 0, 0]

    if args.gui:
        from mrws.io.gui import launch_gui

        os.environ["ROBOTSIM_TRANSMIT"] = "False"

        def make_warehouse():
            return Warehouse(
                args.warehouse, args.items, 3, args.mode, perfect_scenario, True, 1000,
            )

        simu = make_warehouse()
        launch_gui(simu, warehouse_factory=make_warehouse)
    else:
        sim = Simulation(
            args.num_sims,
            args.warehouse,
            args.items,
            3,
            args.mode,
            perfect_scenario,
            True,
            1000,
        )
        sim.run_simulation(True, args.transmit)
