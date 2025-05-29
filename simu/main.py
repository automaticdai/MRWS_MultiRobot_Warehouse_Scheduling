import customexceptions
import warehouse
import robot
import os
import argparse
import time
import statistics
import shutil
import random
import math
import matplotlib

from operator import add

class Simulation:
    def __init__(self, num_sims:int, whouse:str, num_items:int, inv_size:int, schedule_mode:str, fault_rates, fault_mode, step_limit, side_len = None):
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
        self.ga_attempts = [0,0,0,0,0]

        self.order_prio = []
        self.order_prio_sorted_completion_time_lists = [[], [], [], [], []]

        self.order_completion_steps_by_num_robots = {}

    def run_simulation(self, reraise_error, slow_for_transmit):
        for sim_num in range(self._num_sims):
            print("Starting sim %s" % sim_num)
            sim_step_times = []
            try:
                simu = warehouse.Warehouse(self.warehouse_file, self._num_items, self._inv_size,
                                           self._schedule_mode, self._fault_rates, self._fault_mode, self._step_limit)
                keep_step = True
                while keep_step:
                    if slow_for_transmit:
                        time.sleep(0.2)
                    before_step_time = time.perf_counter_ns()
                    keep_step = not simu.step()
                    elapsed_time_between = time.perf_counter_ns() - before_step_time
                    sim_step_times.append(elapsed_time_between)

            except customexceptions.SimulationError as err:
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
            self.order_prio = self.order_prio + simu.get_order_manager().return_mapping_prio_to_completion_times()
            self.ga_attempts = list(map(add, simu.get_scheduler().get_ga_attempts(), self.ga_attempts))

            orders_to_amount_robots = simu.get_scheduler().get_order_to_amount_of_robots_assigned()
            order_start_times = simu.get_order_manager().get_order_start_work_times()
            order_finish_times = simu.get_order_manager().get_order_finish_work_times()

            for order_name in orders_to_amount_robots.keys():
                order_total_time = order_finish_times[order_name] - order_start_times[order_name]
                amount_robots = orders_to_amount_robots[order_name]
                if amount_robots not in self.order_completion_steps_by_num_robots.keys():
                    self.order_completion_steps_by_num_robots[amount_robots] = [order_total_time]
                else:
                    self.order_completion_steps_by_num_robots[amount_robots].append(order_total_time)

            self.num_robots = simu.get_number_of_robots()

        for prio_num in range(5):
            for order_list in self.order_prio:
                if order_list[0] == prio_num + 1:
                    self.order_prio_sorted_completion_time_lists[prio_num].append(order_list[1])

    def print_priority_info(self):
        import matplotlib.pyplot as plt
        import numpy as np
        fig = plt.figure(figsize=(8,8))

        for j in range(5):
            if self.order_prio_sorted_completion_time_lists[j]:
                sel = self.order_prio_sorted_completion_time_lists[j]
                print("Average amount of steps for prio %s completion: %s" % (j + 1, statistics.mean(sel)))
        plt.boxplot(self.order_prio_sorted_completion_time_lists)

        ax = plt.gca()
        plt.title("Order prioritisation statistics for scheduling algorithm %s" % self._schedule_mode)
        ax.set_xlabel("Order priority")
        ax.set_ylabel("Amount of steps to complete order after introduction")

        plt.show()

    def print_num_robots_info(self):
        for amount_bots in self.order_completion_steps_by_num_robots.keys():
            print("Average amount of time taken for an order with %s bots: %s" % (
                amount_bots, statistics.mean(self.order_completion_steps_by_num_robots[amount_bots])))

    def print_error_info(self):
        for err in self.error_strings:
            print(err)
        print("%s simulations faulted critically." % len(self.error_strings))
        return self.error_strings

    def print_steps_taken(self):
        print("Simulation %s took an average of %s steps" % (self._schedule_mode, statistics.mean(self.step_amounts)))
        return self.step_amounts

    def print_step_time_info(self):
        print("Max step time %sms, Min step time %sms" % (max(self.step_times) / 10**6, min(self.step_times) / 10**6))
        print("Mean step time %sms" % (statistics.mean(self.step_times) / 10**6))

        return [self.num_robots, self._side_len, statistics.mean(self.step_times) / 10**6]


def gen_nxn_warehouse(robot_num, side_len):
    robots_left_to_place = robot_num
    goals_left_to_place = robot_num
    filename = "wt%sx%sr%s.txt" % (side_len, side_len, robot_num)
    f = open(filename, "a")

    lines = []

    if robots_left_to_place <= side_len - 2:
        robot_section = ["R" for i in range(robots_left_to_place)]
        left_section = ["X" for i in range(side_len - 2 - robots_left_to_place)]
        middle_section = ''.join(robot_section) + ''.join(left_section)
        robots_left_to_place = 0
    else:
        middle_section = ''.join(["R" for i in range(side_len - 2)])
        robots_left_to_place = robots_left_to_place - (side_len - 2)

    first_line = "X%sX\n" % middle_section


    if goals_left_to_place <= side_len - 2:
        goal_section = ["G" for i in range(goals_left_to_place)]
        left_section = ["X" for i in range(side_len - 2 - goals_left_to_place)]
        middle_section = ''.join(goal_section) + ''.join(left_section)
        goals_left_to_place = 0
    else:
        middle_section = ''.join(["G" for i in range(side_len - 2)])
        goals_left_to_place = goals_left_to_place - (side_len - 2)

    last_line = "X%sX" % middle_section

    lines.append(first_line)

    for i in range(side_len - 2):
        line = ''.join(["X" for i in range(side_len)]) + "\n"
        lines.append(line)

    lines.append(last_line)

    for i in range(robots_left_to_place):
        lines[i+1] = "R" + lines[i+1][1:]
        lines[len(lines) - 2 - i] = lines[len(lines) - 2 - i][:-2] + "G\n"
        if i+1 == side_len - 1:
            raise Exception("not enough space for that amount of robots")

    middle_index = ((side_len + 1) // 2) - 1
    one_above = middle_index - 1
    one_below = middle_index + 1

    spacing = ''.join(["X" for i in range((side_len - 7) // 2)])

    lines[one_above] = lines[one_above][0] + spacing + "SSXSS" + spacing + lines[one_above][-2] + "\n"
    lines[middle_index] = lines[middle_index][0] + spacing + "SSXSS" + spacing + lines[middle_index][-2] + "\n"
    lines[one_below] = lines[one_below][0] + spacing + "SSXSS" + spacing + lines[one_below][-2] + "\n"

    for line in lines:
        f.write(line)

    f.close()

    return filename

def run_simulation_performance_test(scheduling_mode:str, robots_max:int, size_max:int, step_limit:int):
    try:
        os.makedirs("tmp")
    except FileExistsError:
        shutil.rmtree("tmp")
        os.makedirs("tmp")
    os.chdir("tmp")
    results = []
    by_sim_size = []
    ctr = 0
    for i in range(7, size_max, 2):
        by_sim_size.append([])
        for j in range(1, robots_max+1):
            print("starting %s %s" % (j, i))
            file_name = gen_nxn_warehouse(j, i)
            sim_1 = Simulation(10, file_name, 12, 3, scheduling_mode,
                     [0, 0, 0, 0], True, step_limit, i)
            sim_1.run_simulation(False, False)
            results.append(sim_1.print_step_time_info())
            by_sim_size[ctr].append(sim_1.print_step_time_info())
        ctr += 1
    print(results)

    import numpy
    import matplotlib.pyplot as plt
    import matplotlib
    import matplotlib.ticker as ticker
    from mpl_toolkits.mplot3d import proj3d

    res = numpy.array(results)
    fig = plt.figure(figsize=(8,8))
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


def run_completion_time_test(fault_rates):
    sim = Simulation(500, "whouse2.txt", 10, 3, "simple",
                     fault_rates, True, 1000)

    sim_1 = Simulation(500, "whouse2.txt", 10, 3, "simple-interrupt",
                       fault_rates, True, 1000)

    sim_2 = Simulation(500, "whouse2.txt", 10, 3, "multi-robot",
                       fault_rates, True, 1000)

    sim_3 = Simulation(500, "whouse2.txt", 10, 3, "multi-robot-genetic",
                       fault_rates, True, 1000)

    sim.run_simulation(False, False)
    sim_1.run_simulation(False, False)
    sim_2.run_simulation(False, False)
    sim_3.run_simulation(False, False)

    steps = [sim.print_steps_taken(), sim_1.print_steps_taken(), sim_2.print_steps_taken(), sim_3.print_steps_taken()]

    import matplotlib.pyplot as plt
    import numpy as np

    fig = plt.figure(figsize=(8, 8))
    plt.boxplot(steps)
    ax = plt.gca()
    plt.title("Amount of time steps to complete simulation for each scheduling mode")
    plt.xticks([1, 2, 3, 4], ['simple', 'simple-interrupt', 'multi-robot', 'multi-robot-genetic'])
    ax.set_xlabel("Scheduling mode")
    ax.set_ylabel("Amount of steps")
    plt.show()

def run_fault_test(scheduling_mode):
    faulty = [0.0001, 0.001, 0.001, 0.001]
    num_sims = 250

    sim = Simulation(num_sims, "whouse2.txt", 10, 3, scheduling_mode,
                     faulty, True, 1500)

    sim_1 = Simulation(num_sims, "whouse2.txt", 10, 3, scheduling_mode,
                       faulty, False, 1500)



    sim.run_simulation(False, False)
    sim_1.run_simulation(False, False)


    all_errors_1 = sim.print_error_info()
    all_errors_2 = sim_1.print_error_info()

    error_types_1 = [len(all_errors_1),0,0,0]
    error_types_2 = [len(all_errors_2),0,0,0]


    for error_message in all_errors_1:
        if "collided" in str(error_message):
            error_types_1[1] += 1
        if "limit" in str(error_message):
            error_types_1[3] += 1
        if "empty" in str(error_message) or "violated" in str(error_message):
            error_types_1[2] += 1

    for error_message in all_errors_2:
        if "collided" in str(error_message):
            error_types_2[1] += 1
        if "limit" in str(error_message):
            error_types_2[3] += 1
        if "empty" in str(error_message) or "violated" in str(error_message):
            error_types_2[2] += 1

    import matplotlib.pyplot as plt
    import numpy as np
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
    for attribute in groups.keys():
        measurement_1 = groups[attribute]

        offset = width * multiplier
        rects = ax.bar(x + offset, measurement_1, width, label=attribute, color=colors[color_ctr])
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




if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", action="store_true", help="Whether or not to transmit UDP packets.")
    args = parser.parse_args()
    os.environ["ROBOTSIM_TRANSMIT"] = str(args.t)
    faulty = [0.0001, 0.001, 0.001, 0.001]
    perfect_scenario = [0, 0, 0, 0]

    sim = Simulation(1, "whouse2.txt", 10, 3, "simple-interrupt",
                     perfect_scenario, True, 1000)

    sim.run_simulation(True,True)
    #sim.print_priority_info()









