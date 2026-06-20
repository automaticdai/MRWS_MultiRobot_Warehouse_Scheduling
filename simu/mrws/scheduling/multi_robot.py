import copy
import math
import random
from collections import deque

import pygad

from mrws.exceptions import SimulationError
from mrws.models.order import Order
from mrws.scheduling.ga_handler import (
    GAHandler, fitness_func, encode_string_utf8_to_int, decode_utf8_int_to_string
)


def split_items_among_robots(item_names, num_robots, inventory_size):
    """Distribute item names across robots in contiguous chunks of at most
    inventory_size items, one chunk per robot.

    When num_robots == ceil(len(item_names) / inventory_size) (as computed by
    multi_robot_schedule_simple), every robot receives a non-empty chunk no
    larger than inventory_size; the final robot may receive fewer items.
    """
    groups = [[] for _ in range(num_robots)]
    for index, name in enumerate(item_names):
        groups[index // inventory_size].append(name)
    return groups


def reassign_orders_if_faulted(self):
    orders_to_remove = []
    orders_to_add = []
    for order_id, robot_names in self._order_robots_assignment.items():
        if len(robot_names) == 1:
            robot_obj = self._robots[robot_names[0]]
            if robot_obj.battery_faulted_critical or robot_obj.battery_faulted:
                self._schedule[robot_names[0]] = deque()
                order_to_remove, new_order = self.generate_order_to_complete_fault(order_id)

                orders_to_remove.append(order_to_remove)
                orders_to_add.append(new_order)
        else:
            critical_battery_fault_bots = []
            battery_charge_bots = []
            non_faulted_bots = []
            for robot_name in robot_names:
                robot_obj = self._robots[robot_name]
                if robot_obj.battery_faulted_critical:
                    critical_battery_fault_bots.append(robot_name)
                if robot_obj.battery_faulted:
                    battery_charge_bots.append(robot_name)
                if not robot_obj.battery_faulted_critical and not robot_obj.battery_faulted:
                    non_faulted_bots.append(robot_name)
                else:
                    self._schedule[robot_name] = deque()
            if len(critical_battery_fault_bots) > 0 or len(battery_charge_bots) > 0:
                for robot_name in non_faulted_bots:
                    robot_obj = self._robots[robot_name]
                    robot_obj.gone_home_to_clear_inv = True
                    robot_obj.set_target(self._homes[self.get_home_name_for_robot_name(robot_obj.get_name())])
                    self._schedule[robot_obj.get_name()] = deque()
                    order_to_remove, new_order = self.generate_order_to_complete_fault(order_id)

                    orders_to_remove.append(order_to_remove)
                    orders_to_add.append(new_order)

    for order_obj in orders_to_remove:
        removed_order_id = order_obj.get_id()
        removed_robots = self._order_robots_assignment.pop(removed_order_id)
        for rn in removed_robots:
            self._unassign_robot(rn)
        self._orders_active.remove(order_obj)

    for order_obj in orders_to_add:
        self._orders_backlog.append(order_obj)


def generate_order_to_complete_fault(self, order_id):
    items_already_delivered = self.get_items_already_delivered_for_order(order_id)
    order_to_remove = None
    for order_obj in self._orders_active:
        if order_obj.get_id() == order_id:
            order_to_remove = order_obj

    items_left_to_deliver = copy.deepcopy(order_to_remove.get_original_items())

    for item1 in items_already_delivered:
        items_left_to_deliver.remove(item1)

    new_order = Order(items_left_to_deliver, order_to_remove.get_prio(),
                            order_to_remove.get_id(), order_to_remove.get_original_items())

    if len(items_already_delivered) == 0:
        self._order_goal_assignment.pop(order_id)

    return order_to_remove, new_order


def multi_robot_schedule_simple(self, fault_tolerant_mode):
    if fault_tolerant_mode:
        self.reassign_orders_if_faulted()

    orders_to_move = []
    for order_obj in reversed(sorted(self._orders_backlog, key=lambda order1: order1.get_prio())):
        items_by_dependency = reversed(sorted(order_obj.get_items(), key=lambda itm: itm.get_dependency()))
        order_prio = order_obj.get_prio()
        optimal_robots_required = math.ceil(float(len(order_obj.get_items())) / self._ROBOT_INVENTORY_SIZE)

        free_robots = self.find_free_robots(fault_tolerant_mode)
        free_goal_obj = self.find_goal_for_order(order_obj)

        if free_goal_obj is None:
            continue

        if len(free_robots) >= optimal_robots_required:
            selected_robots = free_robots[:optimal_robots_required]

            self._order_goal_assignment[order_obj.get_id()] = free_goal_obj.get_name()
            robot_name_list = list(map(lambda r: r.get_name(), selected_robots))
            self._order_robots_assignment[order_obj.get_id()] = robot_name_list
            for rn in robot_name_list:
                self._assign_robot_to_order(rn, order_obj.get_id())

            for robot_obj in selected_robots:
                robot_obj.set_assigned_order(order_obj.get_id())
                robot_obj.set_prio(order_prio)

            item_names_by_dependency = [item.get_name() for item in items_by_dependency]
            item_names_split_by_bots = split_items_among_robots(
                item_names_by_dependency, len(selected_robots), self._ROBOT_INVENTORY_SIZE)

            robot_ctr = 0
            for item_set in item_names_split_by_bots:
                robot_name = selected_robots[robot_ctr].get_name()
                goal_name = free_goal_obj.get_name()
                for item_name in item_set:
                    shelf_name = self._item_to_shelf_mapping[item_name][0]
                    self.add_to_schedule(robot_name, shelf_name)
                if robot_ctr >= 1:
                    self.add_to_schedule(robot_name, "block|flag%s" % self._mr_flag_ctr)
                    self._mr_flag_ctr += 1

                if robot_ctr != len(item_names_split_by_bots) - 1:
                    self.add_to_schedule(robot_name, "%s|flag%s" % (goal_name, self._mr_flag_ctr))
                else:
                    self.add_to_schedule(robot_name, goal_name)

                robot_ctr += 1
            self._order_to_amount_robots_assigned[order_obj.get_id()] = len(selected_robots)
            orders_to_move.append(order_obj)
        elif len(free_robots) >= 1:
            self.assign_single_robot_schedule_empty_starting_inventory(order_obj,
                                                                       free_robots[0],
                                                                       free_goal_obj)

            self._order_to_amount_robots_assigned[order_obj.get_id()] = 1
            orders_to_move.append(order_obj)

    for ordr in orders_to_move:
        self._orders_backlog.remove(ordr)
        self._orders_active.append(ordr)

    return orders_to_move

def multi_robot_schedule_genetic(self, fault_tolerant_mode):
    # spaghetti alert
    if fault_tolerant_mode:
        self.reassign_orders_if_faulted()

    # Phase 3: GA scaling guard — fall back when too many free robots
    free_robots_check = self.find_free_robots(fault_tolerant_mode)
    if len(free_robots_check) > 50:
        return self.multi_robot_schedule_simple(fault_tolerant_mode)

    orders_to_move = []
    for order_obj in reversed(sorted(self._orders_backlog, key=lambda order1: order1.get_prio())):
        if len(self.find_free_robots(fault_tolerant_mode)) == 0:
            break

        try_counter = 0
        fitness = 0
        while try_counter < 5:
            fitness, original_schedule = self.run_genetic_algorithm(order_obj, fault_tolerant_mode)
            if fitness > 1:
                self._ga_attempts[try_counter] += 1
                break
            else:
                try_counter += 1
        if fitness < 1:
            raise SimulationError("Couldnt find a solution with the genetic algorithm")

        split_indices = []
        seperate_schedules = []
        selected_goal = None
        ctr = 0

        # Work out what robots and goal have been selected by the GA
        for value in original_schedule:
            if "robot" in value:
                split_indices.append(ctr)
            if "goal" in value:
                selected_goal = value.split("|")[0]
            ctr += 1
        split_indices.append(len(original_schedule))

        # Seperate out the schedules into sub-lists
        for i in range(len(split_indices)-1):
            seperate_schedules.append(original_schedule[split_indices[i]:split_indices[i+1]])

        selected_robots = []
        seperate_schedules_dict = {}
        for schedule in seperate_schedules:
            selected_robots.append(schedule[0])
            seperate_schedules_dict[schedule[0]] = schedule[1:]

        # Set the priority of the robots
        for robot_name in selected_robots:
            self._robots[robot_name].set_prio(order_obj.get_prio())

        # Set the order assignment
        self._order_robots_assignment[order_obj.get_id()] = selected_robots
        for rn in selected_robots:
            self._assign_robot_to_order(rn, order_obj.get_id())
        self._order_goal_assignment[order_obj.get_id()] = selected_goal

        # These are the individual pickup trips of each robot.
        pickups = []

        schedule_progress_indices = {}
        robot_visit_order = []

        for robot_name, schedule in seperate_schedules_dict.items():
            schedule_progress_indices[robot_name] = 0
            current_items = []
            for location in schedule:
                if "shelf" in location:
                    current_items.append(self._shelf_to_item_mapping[location])
                if "goal" in location:
                    robot_visit_order.append(robot_name)
                    pickups.append([robot_name, current_items])
                    current_items = []

        if len(pickups) > 1:

            # Sort the pickups by the max item dependency found in each, largest goes first in the list.
            pickups = list(reversed(sorted(pickups, key=lambda outer: max(list(map(lambda inner: inner.get_dependency(),
                                                                              outer[1]))))))

            # However, we also need to make sure that the minimum in each list is sorted by too, if two elements
            # have the same maximum.
            # (Using bubble sort)

            for i in range(len(pickups) - 1):
                this_value = pickups[i]
                next_value = pickups[i+1]
                this_items = this_value[1]
                next_items = next_value[1]
                this_dep_list = list(map(lambda inner: inner.get_dependency(), this_items))
                next_dep_list = list(map(lambda inner: inner.get_dependency(), next_items))
                this_min = min(this_dep_list)
                next_min = min(next_dep_list)
                this_max = max(this_dep_list)
                next_max = max(next_dep_list)
                if this_min < next_min and this_max == next_max:
                    temp_value = this_value
                    pickups[i] = next_value
                    pickups[i+1] = temp_value

        # The pickups list is now sorted correctly into the order in which the pickups have to happen,
        # to fulfill the item dependency.

        pickups = copy.deepcopy(pickups)

        blocks_to_insert = {}
        flags_to_add_on_goals = {}

        # Now we need to figure out where to insert the blocks
        last_robot_name = None
        for item_obj in reversed(sorted(order_obj.get_items(), key= lambda i: i.get_dependency())):
            current_sublist = pickups[0]
            current_robot_name = current_sublist[0]
            current_item_list = current_sublist[1]

            current_schedule = seperate_schedules_dict[current_robot_name]


            if last_robot_name != current_robot_name and last_robot_name != None:

                # Add the flag to the next goal visit, for the previous robot
                hasnt_found_next_goal_yet = True
                last_robot_schedule = seperate_schedules_dict[last_robot_name]
                last_robot_sched_prog = schedule_progress_indices[last_robot_name]

                while hasnt_found_next_goal_yet:
                    if last_robot_sched_prog >= len(last_robot_schedule):
                        raise Exception("Hit end of schedule looking for next goal, for previous robot")

                    loc_found = last_robot_schedule[last_robot_sched_prog]
                    if "goal" in loc_found:
                        hasnt_found_next_goal_yet = False
                        identifier = "%s|%s" % (last_robot_name, last_robot_sched_prog)
                        flags_to_add_on_goals[identifier] = "flag%s" % self._mr_flag_ctr
                    last_robot_sched_prog += 1
                schedule_progress_indices[last_robot_name] = last_robot_sched_prog

                # Add the block before the next goal visit, for this robot
                hasnt_found_next_goal_yet_2 = True
                this_robot_sched_prog = schedule_progress_indices[current_robot_name]
                while hasnt_found_next_goal_yet_2:
                    if this_robot_sched_prog >= len(current_schedule):
                        raise Exception("Hit end of schedule looking for next goal, for this robot")

                    loc_found = current_schedule[this_robot_sched_prog]
                    if "goal" in loc_found:
                        hasnt_found_next_goal_yet_2 = False
                        identifier = "%s|%s" % (current_robot_name, this_robot_sched_prog - 1)
                        blocks_to_insert[identifier] = "block|flag%s" % self._mr_flag_ctr
                    this_robot_sched_prog += 1
                self._mr_flag_ctr += 1
                pass

            hasnt_found_item_yet = True
            while hasnt_found_item_yet:
                if schedule_progress_indices[current_robot_name] == len(current_schedule):
                    raise Exception("Hit end of schedule looking for item %s" % item_obj)
                current_schedule_index = schedule_progress_indices[current_robot_name]
                location_found = current_schedule[current_schedule_index]
                if "shelf" in location_found:
                    if self._shelf_to_item_mapping[location_found] != item_obj:
                        raise Exception("Something has gone wrong in the scheduling")
                    else:
                        hasnt_found_item_yet = False
                schedule_progress_indices[current_robot_name] += 1
            current_item_list.remove(item_obj)
            if len(current_item_list) == 0:
                pickups.pop(0)
            last_robot_name = current_robot_name

        for robot_name, schedule_list in seperate_schedules_dict.items():
            loc_ctr = 0
            for location in schedule_list:
                if "shelf" in location:
                    self.add_to_schedule(robot_name, location)
                for key, value in blocks_to_insert.items():
                    robot_name_block = key.split("|")[0]
                    position = int(key.split("|")[1])
                    if robot_name_block == robot_name and position == loc_ctr:
                        self.add_to_schedule(robot_name, value)
                if "goal" in location:
                    found_flag = False
                    for key, value in flags_to_add_on_goals.items():
                        robot_name_flag = key.split("|")[0]
                        position = int(key.split("|")[1])
                        if robot_name_flag == robot_name and position == loc_ctr:
                            self.add_to_schedule(robot_name, location+"|"+value)
                            found_flag = True
                    if not found_flag:
                        self.add_to_schedule(robot_name, location)
                loc_ctr += 1

        orders_to_move.append(order_obj)

    for ordr in orders_to_move:
        self._orders_backlog.remove(ordr)
        self._orders_active.append(ordr)
    return orders_to_move


def run_genetic_algorithm(self, order_obj, fault_tolerant_mode):

    order_list = list(map(lambda i: i.get_name(), order_obj.get_items()))

    genes = copy.deepcopy(self._all_genes)

    valid_shelves = []
    for item in order_list:
        for shelf in self._item_to_shelf_mapping[item]:
            valid_shelves.append(shelf)

    valid_goal = self.find_goal_for_order(order_obj).get_name()

    free_robots = self.find_free_robots(fault_tolerant_mode)
    free_robot_names = list(map(lambda r: r.get_name(), free_robots))

    genes_to_remove = []


    for gene in genes:
        if ("shelf" in gene) and (gene not in valid_shelves):
            genes_to_remove.append(gene)
        if ("robot" in gene) and (gene not in free_robot_names):
            genes_to_remove.append(gene)
        if ("goal" in gene) and (valid_goal not in gene):
            genes_to_remove.append(gene)

    for gene in genes_to_remove:
        genes.remove(gene)

    genes_no_robots = []
    for gene in genes:
        if "robot" not in gene:
            genes_no_robots.append(gene)

    genes_no_robots_no_shelves = []
    for gene in genes_no_robots:
        if "shelf" not in gene:
            genes_no_robots_no_shelves.append(gene)

    for i in range(len(genes)//2):
        genes.append("wait")

    for i in range(len(genes_no_robots)//2):
        genes_no_robots.append("wait")

    for i in range(len(genes_no_robots_no_shelves)//2):
        genes_no_robots_no_shelves.append("wait")


    h = GAHandler.get_instance()

    h.set_genes(genes_no_robots)

    # 2A: Pass positions for on-demand distance computation instead of precomputed graph
    self._update_robot_positions()
    h.set_positions(self._all_positions)
    h.set_shelf_item_mapping(self._shelf_to_item_mapping)
    h.set_order_to_fulfill(order_list)

    num_parents_mating = 30
    num_genes = max(10, 2*(len(order_list)+2))
    gene_type = int
    gene_space = h.get_int_genes()
    ga_fitness_func = fitness_func
    num_generations = 50

    optimal_robots_required = math.ceil(float(len(order_list)) / self._ROBOT_INVENTORY_SIZE)
    possible_robot_numbers = range(1, min(optimal_robots_required+1, len(free_robots) + 1))

    init_pop = []

    for i in possible_robot_numbers:
        for j in range(200):
            shelf_ctr = 0
            robot_indices = [0]
            offsets = [-2,-1,0,1,2]
            if i > 1:
                for k in range(i - 1):
                    robot_indices.append((((k + 1) * num_genes) // i) - 1 + random.choice(offsets))

            sol = []
            robots_added_to_sol = []
            for k in range(num_genes):
                if k in robot_indices:
                    free_robot_name = random.choice(free_robot_names)
                    while free_robot_name in robots_added_to_sol:
                        free_robot_name = random.choice(free_robot_names)
                    sol.append(free_robot_name)
                    robots_added_to_sol.append(free_robot_name)
                else:
                    if shelf_ctr == len(order_list):
                        gene_choice = random.choice(genes_no_robots_no_shelves)
                        sol.append(gene_choice)
                    else:
                        gene_choice = random.choice(genes_no_robots)
                        if "shelf" in gene_choice:
                            shelf_ctr += 1
                        sol.append(gene_choice)

            enc_sol = []
            for gene in sol:
                enc_sol.append(encode_string_utf8_to_int(gene))
            init_pop.append(enc_sol)


    ga_instance = pygad.GA(num_generations=num_generations,
                           num_genes=num_genes,
                           gene_type=gene_type,
                           gene_space=gene_space,
                           initial_population=init_pop,
                           sol_per_pop=200,
                           fitness_func=ga_fitness_func,
                           num_parents_mating=num_parents_mating)

    ga_instance.run()

    solution, solution_fitness, solution_idx = ga_instance.best_solution()

    flat_schedule = []
    for gene in solution:
        flat_schedule.append(decode_utf8_int_to_string(gene))

    return solution_fitness, flat_schedule
