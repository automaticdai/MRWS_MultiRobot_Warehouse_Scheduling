import math
import random
import time

from mrws.exceptions import SimulationError
from mrws.models.item import Item
from mrws.io import udp
from mrws.entities.shelf import Shelf
from mrws.entities.order_station import OrderStation
from mrws.entities.robot import Robot
from mrws.entities.home import RobotHome
from mrws.engine.order_manager import OrderManager
from mrws.scheduling.scheduler import Scheduler
from mrws.engine.pathfinding import compute_astar_path
from mrws.engine import deadlock
from mrws.utils import robot_prio_sort_key

class Warehouse:
    def __init__(self, w_house_filename: str, num_items: int, robot_max_inventory: int, schedule_mode: str,
                 robot_fault_rates: list[float], fault_tolerant_mode, step_limit: int):
        self._fault_tolerant_mode = fault_tolerant_mode

        self._current_orders = []
        self._cells = []
        self._robots = {}
        self._order_stations = {}
        self._shelves = {}
        self._homes = {}
        self._position_to_robot = {}

        self._robot_fault_rates = robot_fault_rates
        self._robot_max_inventory = robot_max_inventory

        # Generate items first (parse_warehouse_file assigns them to shelves)
        self._items = {}
        self._NUM_ITEMS = num_items
        udp.transmit_start()
        self.generate_items(self._NUM_ITEMS)

        # Parse warehouse so entity counts are known before creating orders
        self._cells = self.parse_warehouse_file(w_house_filename)
        self._width = len(self._cells[0])
        self._height = len(self._cells)

        # Derive order parameters from warehouse contents
        num_robots = len(self._robots)
        num_goals = len(self._order_stations)
        num_shelves = len(self._shelves)

        # Order size: bounded by robot inventory and shelf count
        order_size = max(1, min(robot_max_inventory, num_shelves))
        max_orders = max(1, num_shelves // order_size)

        # Initial orders: at least num_robots so every robot can get a task
        num_init_orders = min(max(num_robots, num_goals * 2), max_orders)

        # Dynamic orders: scale with goals, capped by remaining capacity
        num_dynamic_orders = max(1, min(num_goals, max_orders - num_init_orders))
        num_dynamic_orders = max(0, num_dynamic_orders)

        # Dynamic deadline: scale with warehouse area and entity counts
        self._dynamic_deadline = max(100, (self._width + self._height) * num_shelves // max(num_robots, 1))

        self._order_manager = OrderManager(num_init_orders, num_dynamic_orders,
                                           self._items, self._dynamic_deadline,
                                           order_size)

        self._scheduler = Scheduler(self._order_manager,
                                              self._robots, self._shelves, self._order_stations,
                                              self._homes, self._order_manager.get_init_orders(),
                                              schedule_mode,
                                              self._robot_max_inventory, self._fault_tolerant_mode)
        self._scheduler.schedule(1)

        self.transmit_initial_warehouse_layout()
        self._total_steps = 0
        self._step_limit = step_limit

        self.sensor_faulty_bots = {}
        self._faulty_blocked_cells = set()

    def _recompute_faulty_blocked_cells(self):
        self._faulty_blocked_cells = set()
        for faulty_robot in self.sensor_faulty_bots.values():
            pos = faulty_robot.get_position()
            self._faulty_blocked_cells.add((pos[0] + 1, pos[1]))
            self._faulty_blocked_cells.add((pos[0] - 1, pos[1]))
            self._faulty_blocked_cells.add((pos[0], pos[1] + 1))
            self._faulty_blocked_cells.add((pos[0], pos[1] - 1))

    def step(self):
        self._total_steps = self._total_steps + 1

        if self._total_steps > self._step_limit:
            raise SimulationError("Simulation still running after step limit")

        # Precompute faulty blocked cells once per step
        if self.sensor_faulty_bots:
            self._recompute_faulty_blocked_cells()
        else:
            self._faulty_blocked_cells = set()

        self._needs_reschedule = False

        # ============================================UPDATE ROBOTS====================================================
        for robot_obj in self._robots.values():
            # Apply any faults
            fault_list = robot_obj.maybe_introduce_fault()
            self.apply_fault_actions(robot_obj, fault_list)

            # Robots should only take action if they are not waiting
            if robot_obj.get_wait_steps() == 0:
                self.decide_robot_action(robot_obj)
            else:
                should_schedule = robot_obj.decrement_wait_steps()
                if should_schedule:
                    self._needs_reschedule = True

        # Batch deferred schedule calls
        if self._needs_reschedule:
            self._scheduler.schedule(self._total_steps)

        # ============================================ADD DYNAMIC ORDERS==============================================
        new_order = self._order_manager.possibly_introduce_dynamic_order(self._total_steps)
        if new_order is not None:
            #print("INTRODUCING A NEW ORDER ON STEP %s" % self._total_steps)
            self._scheduler.add_order(new_order, self._total_steps)

        # ============================================DISPLAY LAYOUT==================================================
        #self.print_layout_simple()
        #print("===============================================================================")
        return self._scheduler.are_all_orders_complete() and self.get_total_steps() > self._dynamic_deadline + 1

    def decide_robot_action(self, robot_obj):
        if self._fault_tolerant_mode and not robot_obj.sensors_faulted:
            x = robot_obj.get_position()[0]
            y = robot_obj.get_position()[1]
            if self.cell_within_faulty_robot_move_range(x,y):
                next_positions = [(x+1, y),
                                  (x-1, y),
                                  (x, y+1),
                                  (x, y-1)]
                for pos in next_positions:
                    if not self.is_within_grid(pos[0], pos[1]):
                        continue
                    if not self.cell_is_full(pos[0],pos[1]):
                        # Move to a single safe cell out of the faulty robot's
                        # range, then stop. cell_is_full already excludes cells
                        # within faulty range, so the chosen cell is safe.
                        self.update_robot_position(robot_obj.get_name(), pos[0], pos[1])
                        break


        if robot_obj.get_target() is not None:
            # If the robot is traveling towards its home, it should still be treated as idle and available to schedule
            if (type(robot_obj.get_target()) is RobotHome and not
                    robot_obj.battery_faulted and not robot_obj.gone_home_to_clear_inv):
                # Check if the scheduler has a new job for this robot yet
                #print("Robot is waiting for direction, while travelling home")
                self._scheduler.direct_robot(robot_obj)
                if robot_obj.get_wait_steps() != 0:
                    return
            # If the scheduler did have a new job, the robot will begin moving towards that
            # If it didn't, it will keep moving towards its home

            if not robot_obj.is_at_target():
                #print("Robot is trying to move")
                self.move_robot_towards_astar_collision_detect(robot_obj)
            else:
                #print("Robot is interacting with target")
                robot_obj.interact_with_target()

        else:
            #print("Robot is waiting for direction")
            self._scheduler.direct_robot(robot_obj)

    def apply_fault_actions(self, robot_obj: Robot, fault_list):
        if not fault_list:
            return
        if fault_list[0]:
            robot_obj.add_wait_steps(math.inf)
        if fault_list[1]:
            robot_obj.set_target(self._homes["home%s" % robot_obj.get_name()[5:]])
            robot_obj.apply_charge_wait_upon_reaching_home = True
        if fault_list[2]:
            robot_obj.add_wait_steps(20)
        if fault_list[3]:
            self.sensor_faulty_bots[robot_obj.get_name()] = robot_obj
            robot_obj.add_wait_steps(2)
        if True in fault_list:
            self._needs_reschedule = True

    def get_total_steps(self):
        return self._total_steps

    def is_within_grid(self, x, y):
        return (0 <= x <= self._width - 1) and (0 <= y <= self._height - 1)

    def get_scheduler(self):
        return self._scheduler

    def get_order_manager(self):
        return self._order_manager

    def get_number_of_robots(self):
        return len(self._robots.keys())

    def cell_is_full(self, x, y):
        is_near_faulty_robot = False

        if self._fault_tolerant_mode:
            is_near_faulty_robot = self.cell_within_faulty_robot_move_range(x, y)

        return self.cell_contains_robot(x, y) or is_near_faulty_robot

    def cell_contains_robot(self, x, y):
        return (x, y) in self._position_to_robot

    def cell_within_faulty_robot_move_range(self, x, y):
        return (x, y) in self._faulty_blocked_cells

    def move_robot_towards_astar_collision_detect(self, robot_obj):
        # If the robot has no planned path when we ask it to move, it should try and compute one
        if not robot_obj.get_movement_path():
            robot_obj.set_movement_path(self.compute_robot_astar_path(robot_obj))

        can_move = True
        potential_next_position = None
        # The robot might not have found a valid path, it could be blocked in by other robots
        if robot_obj.get_movement_path():
            potential_next_position = robot_obj.get_movement_path()[0]

            # If a robot has moved into the way of a computed path and blocked this robot then it shouldn't move on this
            # step.
            if self.cell_is_full(potential_next_position[0], potential_next_position[1]):
                can_move = False

        # If we have a next position after the end of this, and the robot can move, it should move to it
        # If the robots sensors have faulted, it couldn't figure out whether it can move or not, so it will just move.
        if (can_move or robot_obj.sensors_faulted) and (potential_next_position is not None):
            self.move_robot_next_path_spot(robot_obj)
            return

        if potential_next_position is None:
            #print("Robot %s couldnt pathfind to its target" % robot_obj.get_name())
            self.attempt_resolve_deadlocks(robot_obj)
        else:
            # If the robot has a movement path, but cant move because it was blocked
            #print("A robot %s couldnt move, as it was blocked" % robot_obj.get_name())
            # Find the robot that is blocking this one from moving
            blocking_robot = self.get_robot_at(potential_next_position[0], potential_next_position[1])
            if blocking_robot is None:
                return

            if len(blocking_robot.get_movement_path()) == 0:
                #print("doing nothing, waiting for the blocking robot to be assigned some movement")
                if robot_obj.get_steps_halted() > 2:
                    #print("the blocking robot took to long, looking for an alternate path")
                    # It should take a minimum of two steps to be assigned any movement from not having any
                    # If this robot has waited that long, it should look for an alternate path
                    robot_obj.set_movement_path(self.compute_robot_astar_path(robot_obj))
                    if robot_obj.get_movement_path():
                        self.move_robot_next_path_spot(robot_obj)
                    else:
                        self.attempt_resolve_deadlocks(robot_obj)
                else:
                    robot_obj.increment_steps_halted()
                return # Robot has moved, quit the function

            blocking_robot_next_position = blocking_robot.get_movement_path()[0]

            if blocking_robot_next_position != robot_obj.get_position():
                if random.random() > 0.1:
                    self.move_robot_break_deadlock(robot_obj, [robot_obj])
                #else:
                    #print("doing nothing, waiting for the blocking robot to move as it will get out the way")
            else:
                #print("ATTEMPTING TO RESOLVE DEADLOCK")
                robots_by_prio = reversed(sorted([robot_obj, blocking_robot], key=robot_prio_sort_key))
                is_horizontal = (blocking_robot.get_position()[0] - robot_obj.get_position()[0]) != 0
                self.move_robot_break_deadlock(robot_obj, robots_by_prio, is_horizontal)

    def compute_robot_astar_path(self, robot_obj):
        return compute_astar_path(
            self._width, self._height, self._position_to_robot,
            robot_obj.get_position(), robot_obj.get_target().get_position()
        )

    def transmit_initial_warehouse_layout(self):
        udp.transmit_warehouse_size(self._width, self._height)
        for row in self._cells:
            for cell in row:
                for obj_name in cell:
                    if "robot" in obj_name:
                        self._robots[obj_name].transmit_creation()
                    elif "shelf" in obj_name:
                        self._shelves[obj_name].transmit_creation()
                    elif "goal" in obj_name:
                        self._order_stations[obj_name].transmit_creation()

    def print_layout_simple(self):
        for i in range(len(self._cells[0]) + 2):
            print("-", end="")
        print("")
        for row in reversed(self._cells):
            print("|", end="")
            for cell in row:
                for obj in cell:
                    if "robot" in obj:
                        if len(cell) == 1:
                            if obj in self.sensor_faulty_bots.keys():
                                print("F", end="")
                            else:
                                print("R", end="")
                        else:
                            if obj in self.sensor_faulty_bots.keys():
                                print("f", end="")
                            else:
                                print("r", end="")
                    elif ("shelf" in obj) and (len(cell) == 1):
                        print("S", end="")
                    elif ("goal" in obj) and (len(cell) == 1):
                        print("G", end="")
                    elif ("home" in obj) and (len(cell) == 1):
                        print("H", end="")
                    elif "wall" in obj:
                        print("W", end="")
                if len(cell) == 0:
                    print(" ", end="")
            print("|")
        for i in range(len(self._cells[0]) + 2):
            print("-", end="")
        print("")

    def parse_warehouse_file(self, filename: str):
        robot_name_ctr = 0
        shelf_name_ctr = 0
        goal_name_ctr = 0
        row_ctr = 0

        prev_width = None

        cells_copy = []
        lines = []

        with open(filename, "r") as f:
            for line_raw in f:
                lines.append(line_raw.strip())

        for line in list(reversed(lines)):
            width = len(line)
            if prev_width is None:
                prev_width = width
            else:
                if width != prev_width:
                    raise ValueError("Warehouse file is not a complete rectangle")
            cells_copy.append([])
            col_ctr = 0
            for char in line:
                if char == "R":
                    new_robot_name = "robot%s" % robot_name_ctr
                    new_robot = Robot(new_robot_name, col_ctr, row_ctr, self._robot_max_inventory,
                                            self._robot_fault_rates)
                    self._robots[new_robot_name] = new_robot

                    new_home_name = "home%s" % robot_name_ctr
                    new_home = RobotHome(new_home_name, new_robot_name, col_ctr, row_ctr)
                    self._homes[new_home_name] = new_home

                    cells_copy[row_ctr].append([new_robot_name, new_home_name])
                    self._position_to_robot[(col_ctr, row_ctr)] = new_robot_name
                    robot_name_ctr = robot_name_ctr + 1
                elif char == "S":
                    new_shelf_name = "shelf%s" % shelf_name_ctr
                    possible_item_name = "item%s" % shelf_name_ctr

                    if possible_item_name in self._items.keys():
                        new_shelf = Shelf(col_ctr, row_ctr, new_shelf_name, self._items[possible_item_name])
                    else:
                        new_shelf = Shelf(col_ctr, row_ctr, new_shelf_name)
                    self._shelves[new_shelf_name] = new_shelf

                    cells_copy[row_ctr].append([new_shelf_name])
                    shelf_name_ctr = shelf_name_ctr + 1
                elif char == "G":
                    new_goal_name = "goal%s" % goal_name_ctr
                    new_goal = OrderStation(col_ctr, row_ctr, new_goal_name,
                                                        lambda: self._scheduler,
                                                        lambda: self._order_manager,
                                                        lambda: self._total_steps)
                    self._order_stations[new_goal_name] = new_goal

                    cells_copy[row_ctr].append([new_goal_name])
                    goal_name_ctr = goal_name_ctr + 1
                elif char == "W":
                    cells_copy[row_ctr].append(["wall"])
                else:
                    cells_copy[row_ctr].append([])
                col_ctr = col_ctr + 1
            row_ctr = row_ctr + 1
        if shelf_name_ctr != self._NUM_ITEMS:
            raise Exception("The incorrect amount of shelves were present for the amount of items specified")
        return cells_copy

    def transmit(self):
        udp.transmit_warehouse_size(self._width, self._height)

    def generate_items(self, num_items):
        for i in range(num_items):
            item_name = "item%s" % i
            self._items[item_name] = Item(item_name, i)
            udp.transmit_item_existence(item_name)

    def move_robot_next_path_spot(self, robot_obj):
        next_spot = robot_obj.get_movement_path()[0]
        self.update_robot_position(robot_obj.get_name(), next_spot[0], next_spot[1])
        robot_obj.get_movement_path().popleft()

    def update_robot_position(self, robot_name, new_x, new_y):
        if self.cell_contains_robot(new_x, new_y):
            raise SimulationError("Two robots collided at (%s,%s)" % (new_x, new_y))

        robot_obj = self._robots[robot_name]
        old_x, old_y = robot_obj.get_position()
        robot_obj.set_position(new_x, new_y)
        self._cells[old_y][old_x].remove(robot_name)
        self._cells[new_y][new_x].append(robot_name)
        del self._position_to_robot[(old_x, old_y)]
        self._position_to_robot[(new_x, new_y)] = robot_name
        udp.transmit_robot_position(robot_name, new_x, new_y)

    # Attach deadlock methods
    attempt_resolve_deadlocks = deadlock.attempt_resolve_deadlocks
    get_robot_at = deadlock.get_robot_at
    move_robot_break_deadlock = deadlock.move_robot_break_deadlock
    move_robots_away_from = deadlock.move_robots_away_from
    resolve_boxed_in_deadlock = deadlock.resolve_boxed_in_deadlock
