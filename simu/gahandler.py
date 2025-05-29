import customexceptions
import copy
import math
import entitywithinventory


def encode_string_utf8_to_int(string1):
    bytes_value = string1.encode('utf-8')
    int_value = int.from_bytes(bytes_value, 'little')
    return int_value


def decode_utf8_int_to_string(int1):
    int_val = int(int1)
    recovered_bytes = int_val.to_bytes((int_val.bit_length() + 7) // 8, 'little')
    recovered_string = recovered_bytes.decode('utf-8')
    return recovered_string

def return_equivalent_full_order(orders, inventory):
    for order in orders:
        if len(order) != len(inventory):
            continue

        comp_items = copy.deepcopy(inventory)
        should_continue = False
        for order_item in order:
            if order_item not in comp_items:
                should_continue = True
                break
            else:
                comp_items.remove(order_item)

        if should_continue:
            continue

        return order

def return_how_close(orders, inventories):
    order_size = len(orders[0])
    total_items = order_size * len(orders)
    correct_ctr = 0

    for order in orders:
        for inventory in inventories:
            comp_items = copy.deepcopy(inventory)
            should_break = False

            for order_item in order:
                if order_item in comp_items:
                    correct_ctr += 1
                    comp_items.remove(order_item)
                    should_break = True
                else:
                    correct_ctr -= 1
            if should_break:
                break

    return (float(correct_ctr) / total_items)



def schedules_have_not_completed(schedules):
    for schedule in schedules:
        if not is_schedule_complete(schedule):
            return True
    return False

def is_schedule_complete(schedule):
    if schedule[2] == len(schedule[1]):
        return True
    return False

def fitness_func(ga_instance, solution, solution_idx):
    class MockRobot(entitywithinventory.InventoryEntity):
        def __init__(self, name, inv_size):
            super().__init__(name, inv_size, False)

    class MockGoal(entitywithinventory.InventoryEntity):
        def __init__(self, name):
            super().__init__(name, math.inf, False)

    PENALTY_SAME_ROBOT_MULTIPLE_TIMES = -2
    PENALTY_NO_START_WITH_ROBOT = -3
    PENALTY_NO_ROBOT = -3
    PENALTY_ROBOT_DOES_NOTHING = -2
    PENALTY_NO_GOALS = -3
    PENALTY_MORE_GOALS_THAN_ORDERS = -2

    handler = GAHandler.get_instance()
    solution_decoded = []
    for int_gene in solution:
        decoded = decode_utf8_int_to_string(int_gene)
        if decoded not in handler.get_all_string_genes() and "robot" not in decoded:
            raise customexceptions.SimulationError("Invalid gene in solution %s" % decoded)
        solution_decoded.append(decoded)

    robot_indices = []
    goals_in_solution = []
    robot_counter = {}
    order_to_fulfill = copy.deepcopy(handler.get_order())
    item_mapping = handler.get_shelf_item_mapping()


    ctr = 0
    for string_gene in solution_decoded:
        if "robot" in string_gene:
            robot_indices.append(ctr)
            if string_gene not in robot_counter.keys():
                robot_counter[string_gene] = 1
            else:
                return PENALTY_SAME_ROBOT_MULTIPLE_TIMES
        if "goal" in string_gene:
            if string_gene not in goals_in_solution:
                goals_in_solution.append(string_gene)
        ctr = ctr + 1

    if not robot_indices:
        return PENALTY_NO_ROBOT

    if "robot" not in solution_decoded[0]:
        return PENALTY_NO_START_WITH_ROBOT

    robot_indices.append(len(solution_decoded))

    robot_schedules_sep = []
    for i in range(len(robot_indices)-1):
        robot_schedules_sep.append(solution_decoded[robot_indices[i]:robot_indices[i+1]])

    goalctrs = {}
    for schedule in robot_schedules_sep:
        if len(schedule) == 1:
            return PENALTY_ROBOT_DOES_NOTHING
        found = False
        for place in schedule:
            if "goal" in place:
                if place not in goalctrs.keys():
                    goalctrs[place] = 1
                else:
                    goalctrs[place] += 1
                found = True
        if not found:
            return PENALTY_NO_GOALS

    if len(goalctrs.keys()) > 1:
        return PENALTY_MORE_GOALS_THAN_ORDERS

    mock_robots = []
    mock_goals = {}
    robots_distance_accumulated = []
    robots_last_actual_location = []

    for robot in robot_schedules_sep:
        mock_robots.append(MockRobot(robot[0], handler.get_max_inventory()))
        robots_distance_accumulated.append(0)
        robots_last_actual_location.append(None)

    for goal in goals_in_solution:
        mock_goals[goal] = MockGoal(goal)


    robot_ctr = 0
    schedules_with_execution_progress = []
    for schedule in robot_schedules_sep:
        # [robot id, schedule, progress]
        schedules_with_execution_progress.append([robot_ctr, schedule, 0])
        robot_ctr += 1

    correct_pickups_so_far = 0
    items_picked_up = []
    schedules_total_length = len(solution_decoded)
    order_total_items = len(order_to_fulfill)
    steps_executed_succesfully = 0
    success = False

    while schedules_have_not_completed(schedules_with_execution_progress):

        smallest_time = math.inf
        smallest_sched = None
        for schedule in schedules_with_execution_progress:
            if robots_distance_accumulated[schedule[0]] < smallest_time and not is_schedule_complete(schedule):
                smallest_time = robots_distance_accumulated[schedule[0]]
                smallest_sched = schedule

        target = smallest_sched[1][smallest_sched[2]]
        robot_id = smallest_sched[0]

        if "shelf" in target:
            try:
                items_picked_up.append(item_mapping[target].get_name())
                mock_robots[robot_id].add_item_to_inventory(item_mapping[target])
                order_items = copy.deepcopy(order_to_fulfill)
                for item in items_picked_up:
                    if item not in order_items:
                        return -1 + (float(correct_pickups_so_far) / order_total_items)**2 + (float(steps_executed_succesfully)/schedules_total_length)**2
                    else:
                        order_items.remove(item)
                correct_pickups_so_far += 1
            except customexceptions.SimulationError:
                return -1 + (float(correct_pickups_so_far) / order_total_items)**2 + (float(steps_executed_succesfully)/schedules_total_length)**2

        elif "goal" in target:
            try:
                mock_goals[target].receive_inventory(mock_robots[robot_id].transfer_inventory())

            except customexceptions.SimulationError as err:
                return -1 + (float(correct_pickups_so_far) / order_total_items)**2 + (float(steps_executed_succesfully)/schedules_total_length)**2

            order1 = return_equivalent_full_order([order_to_fulfill],
                                                  mock_goals[target].report_inventory_item_names())
            if order1 is not None:
                success = True

        smallest_sched[2] = smallest_sched[2] + 1

        if not is_schedule_complete(smallest_sched):
            next_target = smallest_sched[1][smallest_sched[2]]

            new_accumulated_dist = 0

            if ("wait" != next_target) and ("wait" != target):
                new_accumulated_dist = handler.get_distance_between(target, next_target)

            if ("wait" == target) and ("wait" != next_target):
                new_accumulated_dist = handler.get_distance_between(robots_last_actual_location[robot_id], next_target)

            if (next_target == "wait") and (target != "wait"):
                new_accumulated_dist = 1
                robots_last_actual_location[robot_id] = target

            if (next_target == "wait") and (target == "wait"):
                new_accumulated_dist = 1

            robots_distance_accumulated[robot_id] += new_accumulated_dist

        steps_executed_succesfully += 1
        #print("%s|%s || %s|%s "%(steps_executed_succesfully, schedules_total_length, correct_pickups_so_far, order_total_items))
    if not success:
        inventories = []
        for goal in mock_goals.values():
            inventories.append(goal.report_inventory_item_names())
        return return_how_close([order_to_fulfill], inventories)
    #print("success")
    #print(max(robots_distance_accumulated))

    return 1.0 + 1.0 / max(robots_distance_accumulated)



class GAHandler:
    instance = None

    @staticmethod
    def get_instance():
        if GAHandler.instance is None:
            GAHandler.instance = GAHandler()
        return GAHandler.instance

    def __init__(self):
        self._all_gene_strings = []
        self._all_gene_ints = []
        self._shelf_item_mapping = {}
        self._order = None
        self._robot_max_inv = 3
        self._distance_graph = {}

    def set_genes(self, all_gene_strings):
        self._all_gene_strings = all_gene_strings
        self._all_gene_ints = []
        for string1 in self._all_gene_strings:
            self._all_gene_ints.append(encode_string_utf8_to_int(string1))

    def get_int_genes(self):
        return self._all_gene_ints
    def set_distance_graph(self, graph):
        self._distance_graph = graph

    def set_shelf_item_mapping(self, mapping):
        self._shelf_item_mapping = mapping

    def get_shelf_item_mapping(self):
        return self._shelf_item_mapping

    def set_order_to_fulfill(self, orders):
        self._order = orders

    def get_all_string_genes(self):
        return self._all_gene_strings

    def get_order(self):
        return self._order

    def get_max_inventory(self):
        return self._robot_max_inv

    def get_distance_between(self, name1, name2):
        if name1 == name2:
            return 0
        if (name1, name2) not in self._distance_graph.keys():
            if (name2, name1) not in self._distance_graph.keys():
                tup = [name2, name1]
                print("%s not in distance graph" % tup)
                return None
            else:
                return self._distance_graph[(name2, name1)]
        else:
            return self._distance_graph[(name1, name2)]



