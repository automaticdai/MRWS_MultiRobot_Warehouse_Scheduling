import copy
import math
from collections import deque, Counter

from mrws.exceptions import SimulationError
from mrws.io import transport
from mrws.engine.order_manager import OrderManager

from mrws.scheduling.simple import (
    simple_single_robot_schedule,
    single_interrupt_robot_schedule,
)
from mrws.scheduling.multi_robot import (
    multi_robot_schedule_simple,
    multi_robot_schedule_genetic,
    run_genetic_algorithm,
    reassign_orders_if_faulted,
    generate_order_to_complete_fault,
    emit_load_schedule,
)


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
            raise SimulationError("Invalid scheduling mode provided")

        for shelf_name, shelf in self._shelves.items():
            item_name = shelf.get_item().get_name()
            self._shelf_to_item_mapping[shelf_name] = shelf.get_item()

            if item_name not in self._item_to_shelf_mapping.keys():
                self._item_to_shelf_mapping[item_name] = [shelf_name]
            else:
                self._item_to_shelf_mapping[item_name].append(shelf_name)

        self._goals = goals
        self._flags = []
        self._homes = homes

        self._ROBOT_INVENTORY_SIZE = robot_inventory_size

        self._orders_backlog = []
        self._orders_backlog.extend(init_orders)

        self._orders_active = []

        self._order_robots_assignment = {}
        self._order_goal_assignment = {}

        # 2B: Track assigned robots for O(1) free robot lookup
        self._assigned_robots = set()

        # 2F: Reverse mapping robot_name -> order_id
        self._robot_to_order = {}

        self._schedule = {}

        self._all_positions = {}
        self._all_genes = []

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

    def get_ga_attempts(self):
        return self._ga_attempts

    def _update_robot_positions(self):
        for robot_name, robot_obj in self._robots.items():
            self._all_positions[robot_name] = robot_obj.get_position()

    def add_flag(self, flag: str):
        self._flags.append(flag)

    def schedule(self, step_value):
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

    def get_items_already_delivered_for_order(self, order_id):
        order_goal_name = self._order_goal_assignment[order_id]
        order_goal = self._goals[order_goal_name]
        return order_goal.report_inventory()

    def get_order_to_amount_of_robots_assigned(self):
        return self._order_to_amount_robots_assigned

    def _assign_robot_to_order(self, robot_name, order_id):
        self._assigned_robots.add(robot_name)
        self._robot_to_order[robot_name] = order_id

    def _unassign_robot(self, robot_name):
        self._assigned_robots.discard(robot_name)
        self._robot_to_order.pop(robot_name, None)

    def find_free_robots(self, fault_tolerant_mode):
        free_robots = []
        for robot_name, robot in self._robots.items():
            if fault_tolerant_mode:
                # Check if the robot has critically faulted
                if robot.battery_faulted_critical or robot.battery_faulted or robot.gone_home_to_clear_inv:
                    continue
            if robot_name not in self._assigned_robots:
                free_robots.append(robot)
        return free_robots

    def find_goal_for_order(self, order_obj):
        free_goal_obj = None
        # If this order already has an assigned goal
        if (order_obj.get_id() in self._order_goal_assignment.keys() and order_obj.get_id()
                not in self._order_robots_assignment.keys()):
            goal_name = self._order_goal_assignment[order_obj.get_id()]
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
        self._assign_robot_to_order(robot_name, order_obj.get_id())
        self._order_goal_assignment[order_obj.get_id()] = goal_name

        robot_obj.set_prio(order_obj.get_prio())

        robot_inventory_used = 0

        if not single_item_mode:
            for item in reversed(sorted(order_obj.get_items(), key=lambda itm: itm.get_dependency())):
                if item.get_name() not in self._item_to_shelf_mapping.keys():
                    message = "Scheduling impossible, no shelf exists for item %s" % item.get_name()
                    raise SimulationError(message)
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
                    raise SimulationError(message)
                shelf_name = self._item_to_shelf_mapping[item.get_name()][0]
                self.add_to_schedule(robot_name, shelf_name)
                self.add_to_schedule(robot_name, goal_name)


    def add_to_schedule(self, robot_name, target_name):
        if robot_name not in self._schedule:
            self._schedule[robot_name] = deque()
        self._schedule[robot_name].append(target_name)

    def prepend_to_schedule(self, robot_name, targets_list):
        if robot_name not in self._schedule:
            self._schedule[robot_name] = deque()
        for item in reversed(targets_list):
            self._schedule[robot_name].appendleft(item)

    def add_order(self, order, step_value):
        self._orders_backlog.append(order)
        self.schedule(step_value)

    def direct_robot(self, robot_obj):
        robot_name = robot_obj.get_name()
        if robot_name in self._schedule:
            # We shouldn't do anything if the scheduler has nothing more for this robot
            if self._schedule[robot_name]:
                robot_next_target_name = self._schedule[robot_name].popleft()
                robot_next_target_obj = self.parse_schedule_value(robot_next_target_name, robot_obj)
                robot_obj.set_target(robot_next_target_obj)
            else:
                # 2F: Use reverse mapping for O(1) lookup
                order_id = self._robot_to_order.get(robot_name)
                if order_id is not None and order_id in self._order_robots_assignment:
                    robot_assignment = self._order_robots_assignment[order_id]
                    if robot_name in robot_assignment:
                        robot_assignment.remove(robot_name)
                self._unassign_robot(robot_name)
                selected_home = self._homes[self.get_home_name_for_robot_name(robot_name)]
                self._schedule[robot_name] = deque([selected_home.get_name()])

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
                robot_next_target_obj = self.parse_schedule_value(self._schedule[robot_obj.get_name()].popleft(), robot_obj)
            else:
                robot_next_target_obj = self._homes[self.get_home_name_for_robot_name(robot_obj.get_name())]
                self.prepend_to_schedule(robot_obj.get_name(), [robot_next_target_name])
        elif "wait" in dest_type:
            robot_next_target_obj = self._homes[self.get_home_name_for_robot_name(robot_obj.get_name())]
            robot_obj.add_wait_steps(2)

        if robot_next_target_obj is None:
            message = "Invalid target %s in schedule for robot %s" % (robot_next_target_name, robot_obj.get_name())
            raise SimulationError(message)

        return robot_next_target_obj

    def get_home_name_for_robot_name(self, robot_name):
        return "home%s" % robot_name[5:]

    def are_all_orders_complete(self):
        if self._orders_backlog:
            return False
        if self._orders_active:
            return False
        return True

    def is_this_a_complete_order(self, items: list, order_manager: OrderManager, robot_obj, goal_name, step_ctr):
        # 2E: Use Counter for O(A*I) matching instead of deepcopy + O(A*I²) list matching
        items_counter = Counter(i.get_name() for i in items)

        for order in self._orders_active:
            order_items = order.get_original_items()
            order_counter = Counter(i.get_name() for i in order_items)

            if order_counter != items_counter:
                continue

            if robot_obj.get_name() in self._order_robots_assignment[order.get_id()] and goal_name == self._order_goal_assignment[order.get_id()]:

                self._orders_active.remove(order)

                robots = self._order_robots_assignment.pop(order.get_id())
                for rn in robots:
                    self._unassign_robot(rn)
                self._order_goal_assignment.pop(order.get_id())
                order_manager.set_order_completion_time(order, step_ctr)
                transport.transmit_order_complete(order.get_id())

                self.schedule(step_ctr)

                return True
        return False

    # Attach split-out methods
    simple_single_robot_schedule = simple_single_robot_schedule
    single_interrupt_robot_schedule = single_interrupt_robot_schedule
    multi_robot_schedule_simple = multi_robot_schedule_simple
    multi_robot_schedule_genetic = multi_robot_schedule_genetic
    run_genetic_algorithm = run_genetic_algorithm
    reassign_orders_if_faulted = reassign_orders_if_faulted
    generate_order_to_complete_fault = generate_order_to_complete_fault
    emit_load_schedule = emit_load_schedule
