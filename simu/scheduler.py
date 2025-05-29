import copy
import math


import customexceptions
import gahandler
import ordermanager
import utils
import pygad
import order
import random

class Scheduler:
    def __init__(self, order_manager, robots: dict, shelves: dict, goals: dict, homes: dict, init_orders: list, schedule_mode:str,
                 robot_inventory_size: int,
                 fault_tolerant_mode: bool):
        self._order_manager_ref = order_manager
        self._fault_tolerant_mode = fault_tolerant_mode
        self._robots = robots
        self._num_robots = len(robots.keys())
        self._shelves = shelves
        self._item_to_shelf_mapping = {}
        self._shelf_to_item_mapping = {}

        self._order_to_amount_robots_assigned = {}

        self._schedule_mode = schedule_mode
        self._ga_attempts = [0,0,0,0,0]

        if schedule_mode not in ["simple", "simple-interrupt", "multi-robot", "multi-robot-genetic"]:
            raise customexceptions.SimulationError("Invalid scheduling mode provided")

        for shelf_name, shelf in self._shelves.items():
            item_name = shelf.get_item().get_name()
            self._shelf_to_item_mapping[shelf_name] = shelf.get_item()

            if item_name not in self._item_to_shelf_mapping.keys():
                self._item_to_shelf_mapping[item_name] = [shelf_name]
            else:
                self._item_to_shelf_mapping[item_name] = self._item_to_shelf_mapping[item_name].append(shelf_name)

        self._goals = goals
        self._flags = []
        self._homes = homes

        self._ROBOT_INVENTORY_SIZE = robot_inventory_size

        self._orders_backlog = []
        self._orders_backlog.extend(init_orders)

        self._orders_active = []

        self._order_robots_assignment = {}
        self._order_goal_assignment = {}

        self._schedule = {}

        self._all_positions = {}
        self._all_genes = []
        self._all_distances = {}

        self._mr_flag_ctr = 0

        for robot_name, robot_obj in self._robots.items():
            self._all_positions[robot_name] = robot_obj.get_position()
            self._all_genes.append(robot_name)

        for shelf_name, shelf_obj in self._shelves.items():
            self._all_positions[shelf_name] = shelf_obj.get_position()
            self._all_genes.append(shelf_name)

        for goal_name, goal_obj in self._goals.items():
            self._all_positions[goal_name] = goal_obj.get_position()
            self._all_genes.append(goal_name)

        self.recalculate_distances()

    def get_ga_attempts(self):
        return self._ga_attempts

    def recalculate_distances(self):
        for robot_name, robot_obj in self._robots.items():
            self._all_positions[robot_name] = robot_obj.get_position()

        for location1 in self._all_positions.keys():
            for location2 in self._all_positions.keys():
                if (location2, location1) not in self._all_distances.keys():
                    if (((location1, location2) not in self._all_distances.keys()) or
                            ("robot" in location1) or ("robot" in location2)):
                        if location1 != location2:
                            x1 = self._all_positions[location1][0]
                            y1 = self._all_positions[location1][1]

                            x2 = self._all_positions[location2][0]
                            y2 = self._all_positions[location2][1]

                            self._all_distances[(location1, location2)] = utils.taxicab_dist(x1,y1,x2,y2)

    def add_flag(self, flag: str):
        self._flags.append(flag)

    def schedule(self, step_value):
        #print("SCHEDULING")
        #print("The current backlog is %s" % self._orders_backlog)
        #print("The current active is %s" % self._orders_active)
        #print("The current robot assignment is %s" % self._order_robots_assignment)
        #print("The current goal assignment is %s" % self._order_goal_assignment)

        new_orders = []
        if self._schedule_mode == "simple":
            new_orders = self.simple_single_robot_schedule(self._fault_tolerant_mode)
        elif self._schedule_mode == "simple-interrupt":
            new_orders = self.single_interrupt_robot_schedule(self._fault_tolerant_mode)
        elif self._schedule_mode == "multi-robot":
            new_orders = self.multi_robot_schedule_simple(self._fault_tolerant_mode)
        elif self._schedule_mode == "multi-robot-genetic":
            new_orders = self.multi_robot_schedule_genetic(self._fault_tolerant_mode)

        for order_obj in new_orders:
            self._order_manager_ref.set_order_start_work_time(order_obj.get_id(), step_value)

        #print("AFTER SCHEDULING")
        #print("After, the current backlog is %s" % self._orders_backlog)
        #print("After, the current active is %s" % self._orders_active)
        #print("After, The current robot assignment is %s" % self._order_robots_assignment)
        #print("After, The current goal assignment is %s" % self._order_goal_assignment)
    def get_items_already_delivered_for_order(self, order_id):
        order_goal_name = self._order_goal_assignment[order_id]
        order_goal = self._goals[order_goal_name]
        return order_goal.report_inventory()

    def get_order_to_amount_of_robots_assigned(self):
        return self._order_to_amount_robots_assigned

    def reassign_orders_if_faulted(self):
        orders_to_remove = []
        orders_to_add = []
        for order_id, robot_names in self._order_robots_assignment.items():
            if len(robot_names) == 1:
                robot_obj = self._robots[robot_names[0]]
                if robot_obj.battery_faulted_critical or robot_obj.battery_faulted:
                    self._schedule[robot_names[0]] = []
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
                        self._schedule[robot_name] = []
                if len(critical_battery_fault_bots) > 0 or len(battery_charge_bots) > 0:
                    for robot_name in non_faulted_bots:
                        robot_obj = self._robots[robot_name]
                        robot_obj.gone_home_to_clear_inv = True
                        robot_obj.set_target(self._homes[self.get_home_name_for_robot_name(robot_obj.get_name())])
                        self._schedule[robot_obj.get_name()] = []
                        order_to_remove, new_order = self.generate_order_to_complete_fault(order_id)

                        orders_to_remove.append(order_to_remove)
                        orders_to_add.append(new_order)

        for order_obj in orders_to_remove:
            self._order_robots_assignment.pop(order_obj.get_id())
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

        new_order = order.Order(items_left_to_deliver, order_to_remove.get_prio(),
                                order_to_remove.get_id(), order_to_remove.get_original_items())

        if len(items_already_delivered) == 0:
            self._order_goal_assignment.pop(order_id)

        return order_to_remove, new_order

    def simple_single_robot_schedule(self, fault_tolerant_mode, single_item_mode=False):
        if fault_tolerant_mode:
            self.reassign_orders_if_faulted()

        orders_to_move = []
        # For every order in the backlog (sorted by priority)
        for order_obj in reversed(sorted(self._orders_backlog, key=lambda order1: order1.get_prio())):

            free_robots = self.find_free_robots(fault_tolerant_mode)
            if not free_robots:
                free_robot_obj = None
            else:
                free_robot_obj = free_robots[0]

            free_goal_obj = self.find_goal_for_order(order_obj)

            if (free_robot_obj is not None) and (free_goal_obj is not None):
                self.assign_single_robot_schedule_empty_starting_inventory(order_obj,
                                                                           free_robot_obj,
                                                                           free_goal_obj,
                                                                           single_item_mode)
                orders_to_move.append(order_obj)

        for ordr in orders_to_move:
            self._orders_backlog.remove(ordr)
            self._orders_active.append(ordr)
        return orders_to_move

    def single_interrupt_robot_schedule(self, fault_tolerant_mode):
        if fault_tolerant_mode:
            self.reassign_orders_if_faulted()

        orders_to_move = []
        for order_obj in reversed(sorted(self._orders_backlog, key=lambda order1: order1.get_prio())):
            free_robots = self.find_free_robots(fault_tolerant_mode)
            if not free_robots:
                free_robot_obj = None
            else:
                free_robot_obj = free_robots[0]

            free_goal_obj = self.find_goal_for_order(order_obj)

            if (free_robot_obj is not None) and (free_goal_obj is not None):
                self.assign_single_robot_schedule_empty_starting_inventory(order_obj, free_robot_obj, free_goal_obj)
                orders_to_move.append(order_obj)

            elif free_robot_obj is None and free_goal_obj is not None:
                found_lower_prio_robot_name = None
                lowest_prio_found = order_obj.get_prio()

                for order_id, robot_names in self._order_robots_assignment.items():
                    if len(robot_names) != 1:
                        raise customexceptions.SimulationError("Only one robot should be assigned to each order for this scheduling type")
                    robot_obj = self._robots[robot_names[0]]
                    robot_prio = robot_obj.get_prio()

                    if robot_prio < lowest_prio_found:
                        target_good = False
                        inventory_usage_good = False

                        if robot_obj.get_target() is None:
                            target_good = True
                        else:
                            # We don't want a robot with a shelf as its target as its inventory size will increase
                            # by one and invalidate the following calculations
                            if "shelf" not in robot_obj.get_target().get_name():
                                target_good = True

                        if robot_obj.get_inventory_usage() != 0:
                            if robot_obj.get_inventory_usage() != self._ROBOT_INVENTORY_SIZE:
                                if order_obj.get_highest_item_dep() <= robot_obj.peek_inventory().get_dependency():
                                    inventory_usage_good = True
                        else:
                            inventory_usage_good = True

                        if inventory_usage_good and target_good:
                            lowest_prio_found = robot_prio
                            found_lower_prio_robot_name = robot_names[0]

                if found_lower_prio_robot_name is not None:
                    #print("Order ID is %s" % order_obj.get_id())
                    #print("New goal is %s" % free_goal_obj.get_name())
                    selected_bot = self._robots[found_lower_prio_robot_name]
                    #print(selected_bot.get_inventory_usage())
                    self._order_robots_assignment[order_obj.get_id()] = [found_lower_prio_robot_name]
                    selected_bot.set_prio(order_obj.get_prio())
                    self._order_goal_assignment[order_obj.get_id()] = free_goal_obj.get_name()

                    robot_inventory_already_used = selected_bot.get_inventory_usage()

                    prepend_schedule = []
                    robot_inventory_used_for_this_order = 0
                    for item in reversed(sorted(order_obj.get_items(), key=lambda itm: itm.get_dependency())):
                        if item.get_name() not in self._item_to_shelf_mapping.keys():
                            message = "Scheduling impossible, no shelf exists for item %s" % item.get_name()
                            raise customexceptions.SimulationError(message)
                        shelf_name = self._item_to_shelf_mapping[item.get_name()][0]
                        if robot_inventory_used_for_this_order == self._ROBOT_INVENTORY_SIZE - robot_inventory_already_used:
                            prepend_schedule.append("%s|%s" % (free_goal_obj.get_name(),
                                                               robot_inventory_used_for_this_order))
                            robot_inventory_used_for_this_order = 0
                        prepend_schedule.append(shelf_name)
                        robot_inventory_used_for_this_order = robot_inventory_used_for_this_order + 1

                    prepend_schedule.append("%s|%s" % (free_goal_obj.get_name(), robot_inventory_used_for_this_order))

                    self.prepend_to_schedule(found_lower_prio_robot_name, prepend_schedule)

                    #print("Interruption schedule complete for robot %s is %s" % (found_lower_prio_robot_name, self._schedule[found_lower_prio_robot_name]))
                    orders_to_move.append(order_obj)

        for ordr in orders_to_move:
            self._orders_backlog.remove(ordr)
            self._orders_active.append(ordr)
        return orders_to_move

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
                item_names_split_by_bots = []

                self._order_goal_assignment[order_obj.get_id()] = free_goal_obj.get_name()
                self._order_robots_assignment[order_obj.get_id()] = list(map(lambda r: r.get_name(), selected_robots))

                for robot_obj in selected_robots:
                    robot_obj.set_assigned_order(order_obj.get_id())
                    robot_obj.set_prio(order_prio)
                    item_names_split_by_bots.append([])

                robot_ctr = 0
                robot_inventory_used = 0
                for item in items_by_dependency:
                    item_names_split_by_bots[robot_ctr].append(item.get_name())
                    robot_inventory_used += 1
                    if robot_inventory_used == self._ROBOT_INVENTORY_SIZE:
                        robot_ctr += 1

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

        #print("multi robot scheduling completed.")
        #print("goal assignment %s" % self._order_goal_assignment)
        #print("robot assignment %s" % self._order_robots_assignment)
        #print("active orders %s" % self._orders_active)
        #for robot_name in self._robots.keys():
            #print("Robot %s: %s" % (robot_name, self._schedule[robot_name]))
        return orders_to_move

    def multi_robot_schedule_genetic(self, fault_tolerant_mode):
        # spaghetti alert
        if fault_tolerant_mode:
            self.reassign_orders_if_faulted()

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
                    #print("That solution wasnt good enough")
                    try_counter += 1
            if fitness < 1:
                raise customexceptions.SimulationError("Couldnt find a solution with the genetic algorithm")
            #print("Entering for order %s" % order_obj.get_id())

            #("Genetic schedule %s" % schedule)
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

            #print(pickups)
            if len(pickups) > 1:

                # Sort the pickups by the max item dependency found in each, largest goes first in the list.
                pickups = list(reversed(sorted(pickups, key=lambda outer: max(list(map(lambda inner: inner.get_dependency(),
                                                                                  outer[1]))))))

                #print(pickups)

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

                #print("final")
                #print(pickups)

            # The pickups list is now sorted correctly into the order in which the pickups have to happen,
            # to fulfill the item dependency.

            pickups = copy.deepcopy(pickups)

            blocks_to_insert = {}
            flags_to_add_on_goals = {}

            # Now we need to figure out where to insert the blocks
            last_robot_name = None
            for item_obj in reversed(sorted(order_obj.get_items(), key= lambda i: i.get_dependency())):
                #print("considering item %s" % item_obj)
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
                #print(pickups)
                last_robot_name = current_robot_name

            #print("The robot visit order must be %s" % robot_visit_order)
            #print("The order was %s" % order_obj.get_items())
            #print("Blocks to insert are %s" % blocks_to_insert)
            #print("Flags to insert are %s" % flags_to_add_on_goals)
            #print(original_schedule)

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

                #print("Added schedule for robot %s: %s" % (robot_name, self._schedule[robot_name]))
            #print("FOR ORDER %s" % order_obj.get_id())
            orders_to_move.append(order_obj)

        for ordr in orders_to_move:
            self._orders_backlog.remove(ordr)
            self._orders_active.append(ordr)
        return orders_to_move




    def find_free_robots(self, fault_tolerant_mode):
        free_robots = []
        for robot_name, robot in self._robots.items():
            if fault_tolerant_mode:
                # Check if the robot has critically faulted
                if robot.battery_faulted_critical or robot.battery_faulted or robot.gone_home_to_clear_inv:
                    continue
            # Check whether there is a free robot to take the order
            robot_already_used = False
            for assignment in self._order_robots_assignment.values():
                if robot_name in assignment:
                    robot_already_used = True

            if not robot_already_used:
                free_robots.append(robot)
        return free_robots

    def find_goal_for_order(self, order_obj):
        free_goal_obj = None
        # If this order already has an assigned goal
        if (order_obj.get_id() in self._order_goal_assignment.keys() and order_obj.get_id()
                not in self._order_robots_assignment.keys()):
            goal_name = self._order_goal_assignment[order_obj.get_id()]
            #print("preserving goal for order %s" % order_obj.get_id())
            # Then we can use the same goal again
            free_goal_obj = self._goals[goal_name]
        else:
            # Otherwise, for every goal
            for goal_name, goal in self._goals.items():

                # Check whether its being used
                if goal_name not in self._order_goal_assignment.values():
                    free_goal_obj = goal

        return free_goal_obj

    def assign_single_robot_schedule_empty_starting_inventory(self, order_obj, robot_obj, goal_obj, single_item_mode = False):

        robot_name = robot_obj.get_name()
        goal_name = goal_obj.get_name()
        self._order_robots_assignment[order_obj.get_id()] = [robot_name]
        self._order_goal_assignment[order_obj.get_id()] = goal_name

        robot_obj.set_prio(order_obj.get_prio())
        #print("order %s assigned to robot %s" % (order_obj.get_id(), robot_name))
        #print("using goal %s" % goal_name)

        robot_inventory_used = 0

        if not single_item_mode:
            for item in reversed(sorted(order_obj.get_items(), key=lambda itm: itm.get_dependency())):
                if item.get_name() not in self._item_to_shelf_mapping.keys():
                    message = "Scheduling impossible, no shelf exists for item %s" % item.get_name()
                    raise customexceptions.SimulationError(message)
                shelf_name = self._item_to_shelf_mapping[item.get_name()][0]
                if robot_inventory_used == self._ROBOT_INVENTORY_SIZE:
                    self.add_to_schedule(robot_name, goal_name)
                    robot_inventory_used = 0
                self.add_to_schedule(robot_name, shelf_name)
                robot_inventory_used = robot_inventory_used + 1
            self.add_to_schedule(robot_name, goal_name)
        else:
            for item in reversed(sorted(order_obj.get_items(), key=lambda itm: itm.get_dependency())):
                if item.get_name() not in self._item_to_shelf_mapping.keys():
                    message = "Scheduling impossible, no shelf exists for item %s" % item.get_name()
                    raise customexceptions.SimulationError(message)
                shelf_name = self._item_to_shelf_mapping[item.get_name()][0]
                self.add_to_schedule(robot_name, shelf_name)
                self.add_to_schedule(robot_name, goal_name)

        #print("its complete schedule is %s" % self._schedule[robot_name])




    def add_to_schedule(self, robot_name, target_name):
        if robot_name not in self._schedule.keys():
            self._schedule[robot_name] = []
        self._schedule[robot_name].append(target_name)

    def prepend_to_schedule(self, robot_name, targets_list):
        if robot_name not in self._schedule.keys():
            self._schedule[robot_name] = []
        self._schedule[robot_name] = targets_list + self._schedule[robot_name]

    def add_order(self, order, step_value):
        self._orders_backlog.append(order)
        self.schedule(step_value)

    def direct_robot(self, robot_obj):
        robot_name = robot_obj.get_name()
        if robot_name in self._schedule.keys():
            # We shouldn't do anything if the scheduler has nothing more for this robot
            if self._schedule[robot_name]:
                #print("my schedule was %s" % self._schedule[robot_name])
                robot_next_target_name = self._schedule[robot_name].pop(0)
                #print("popping %s" % robot_next_target_name)
                robot_next_target_obj = self.parse_schedule_value(robot_next_target_name, robot_obj)
                robot_obj.set_target(robot_next_target_obj)
            else:
                for robot_assignment in self._order_robots_assignment.values():
                    if robot_name in robot_assignment:
                        robot_assignment.remove(robot_name)
                        # homeN is robotN's home
                #print("setting %s to return home" % robot_name)
                selected_home = self._homes[self.get_home_name_for_robot_name(robot_name)]
                self._schedule[robot_name] = [selected_home.get_name()]

    def parse_schedule_value(self, robot_next_target_name, robot_obj):
        robot_next_target_obj = None
        dest_type = robot_next_target_name.split("|")[0]
        if "shelf" in dest_type:
            robot_next_target_obj = self._shelves[robot_next_target_name]
        elif "goal" in dest_type:
            if "|" in robot_next_target_name:
                if "flag" in robot_next_target_name:
                    goal_name = robot_next_target_name.split("|")[0]
                    flag_name = robot_next_target_name.split("|")[1]
                    robot_next_target_obj = self._goals[goal_name]
                    robot_obj.set_flag(flag_name)
                else:
                    goal_name = robot_next_target_name.split("|")[0]
                    amount_items = int(robot_next_target_name.split("|")[1])

                    robot_next_target_obj = self._goals[goal_name]
                    robot_obj.set_amount_of_items_to_transfer_next_time(amount_items)
            else:
                robot_next_target_obj = self._goals[robot_next_target_name]
        elif "home" in dest_type:
            robot_next_target_obj = self._homes[robot_next_target_name]
        elif "block" in dest_type:
            split = robot_next_target_name.split("|")
            flag_name = split[1]

            if flag_name in self._flags:
                robot_next_target_obj = self.parse_schedule_value(self._schedule[robot_obj.get_name()].pop(0), robot_obj)
            else:
                robot_next_target_obj = self._homes[self.get_home_name_for_robot_name(robot_obj.get_name())]
                self.prepend_to_schedule(robot_obj.get_name(), [robot_next_target_name])
        elif "wait" in dest_type:
            robot_next_target_obj = self._homes[self.get_home_name_for_robot_name(robot_obj.get_name())]
            robot_obj.add_wait_steps(2)

        if robot_next_target_obj is None:
            message = "Invalid target %s in schedule for robot %s" % (robot_next_target_name, robot_obj.get_name())
            raise customexceptions.SimulationError(message)

        return robot_next_target_obj

    def get_home_name_for_robot_name(self, robot_name):
        return "home%s" % robot_name[5:]

    def are_all_orders_complete(self):
        if self._orders_backlog:
            return False
        if self._orders_active:
            return False
        return True

    def is_this_a_complete_order(self, items: list, order_manager: ordermanager.OrderManager, robot_obj, goal_name, step_ctr):
        for order in self._orders_active:
            comp_items = copy.deepcopy(items)
            should_continue = False
            for order_item in order.get_original_items():
                if order_item not in comp_items:
                    should_continue = True
                    break
                else:
                    comp_items.remove(order_item)

            if should_continue:
                continue

            if len(comp_items) == 0 and robot_obj.get_name() in self._order_robots_assignment[order.get_id()] and goal_name == self._order_goal_assignment[order.get_id()]:

                self._orders_active.remove(order)

                robots = self._order_robots_assignment.pop(order.get_id())
                self._order_goal_assignment.pop(order.get_id())
                #print("Order %s completed by robot %s" % (order.get_id(), robots))
                #print("order %s complete" % order.get_id())
                order_manager.set_order_completion_time(order, step_ctr)

                self.schedule(step_ctr)

                return True
        return False


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


        h = gahandler.GAHandler.get_instance()

        h.set_genes(genes_no_robots)

        self.recalculate_distances()
        h.set_distance_graph(self._all_distances)
        h.set_shelf_item_mapping(self._shelf_to_item_mapping)
        h.set_order_to_fulfill(order_list)

        num_parents_mating = 30
        num_genes = max(10, 2*(len(order_list)+2))
        gene_type = int
        gene_space = h.get_int_genes()
        fitness_func = gahandler.fitness_func
        num_generations = 50

        optimal_robots_required = math.ceil(float(len(order_list)) / self._ROBOT_INVENTORY_SIZE)
        possible_robot_numbers = range(1, min(optimal_robots_required+1, len(free_robots) + 1))

        init_pop = []

        #print("The order is %s" % order_list)

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

                #print(sol)
                enc_sol = []
                for gene in sol:
                    enc_sol.append(gahandler.encode_string_utf8_to_int(gene))
                init_pop.append(enc_sol)


        ga_instance = pygad.GA(num_generations=num_generations,
                               num_genes=num_genes,
                               gene_type=gene_type,
                               gene_space=gene_space,
                               initial_population=init_pop,
                               sol_per_pop=200,
                               fitness_func=fitness_func,
                               num_parents_mating=num_parents_mating)

        ga_instance.run()

        solution, solution_fitness, solution_idx = ga_instance.best_solution()
        #print("best:")
        #print(solution_fitness)
        #print("There are %s free robots left" % len(free_robots))
        #for i in possible_robot_numbers:
           #print(i)

        flat_schedule = []
        for gene in solution:
            flat_schedule.append(gahandler.decode_utf8_int_to_string(gene))

        #ga_instance.plot_fitness()

        return solution_fitness, flat_schedule












