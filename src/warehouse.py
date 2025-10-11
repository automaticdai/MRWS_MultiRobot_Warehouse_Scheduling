import math
import random

import customexceptions
import item
import udptransmit
import shelf
import orderstation
import robot
import heapq
import utils
import ordermanager
import scheduler
import robothome
import time
from dataclasses import dataclass, field

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

        self._robot_fault_rates = robot_fault_rates

        self._items = {}
        self._NUM_ITEMS = num_items
        udptransmit.transmit_start()
        self.generate_items(self._NUM_ITEMS)
        self._dynamic_deadline = 100

        self._order_manager = ordermanager.OrderManager(5, 5, self._items,
                                                        self._dynamic_deadline)

        self._robot_max_inventory = robot_max_inventory
        # Warehouse cell (x,y) is accessed via self._cells[y][x]
        self._cells = self.parse_warehouse_file(w_house_filename)

        self._scheduler = scheduler.Scheduler(self._order_manager,
                                              self._robots, self._shelves, self._order_stations,
                                              self._homes, self._order_manager.get_init_orders(),
                                              schedule_mode,
                                              self._robot_max_inventory, self._fault_tolerant_mode)
        self._scheduler.schedule(1)

        self._width = len(self._cells[0])
        self._height = len(self._cells)

        self.transmit_initial_warehouse_layout()
        self._total_steps = 0
        self._step_limit = step_limit

        self.sensor_faulty_bots = {}

    def step(self):
        self._total_steps = self._total_steps + 1

        if self._total_steps > self._step_limit:
            raise customexceptions.SimulationError("Simulation still running after step limit")

        # ============================================UPDATE ROBOTS====================================================
        for robot_obj in self._robots.values():
            # Apply any faults
            fault_list = robot_obj.maybe_introduce_fault()
            self.apply_fault_actions(robot_obj, fault_list)



            # Robots should only take action if they are not waiting
            if robot_obj.get_wait_steps() == 0:
                #print("Updating robot %s" % robot_obj.get_name())
                #print("At (%s %s)" % (robot_obj.get_position()[0], robot_obj.get_position()[1]))
                #print("Has target %s" % robot_obj.get_target())
                self.decide_robot_action(robot_obj)
            else:
                #print("Robot %s waited a step" % robot_obj.get_name())
                should_schedule = robot_obj.decrement_wait_steps()
                if should_schedule:
                    self._scheduler.schedule(self._total_steps)
            #self.print_layout_simple()
            #print(self._scheduler._orders_active)
            #print(self._scheduler._orders_backlog)
            #for value in self._order_stations.values():
                #print(value.report_inventory())

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
                        self.update_robot_position(robot_obj.get_name(), pos[0], pos[1])


        if robot_obj.get_target() is not None:
            # If the robot is traveling towards its home, it should still be treated as idle and available to schedule
            if (type(robot_obj.get_target()) is robothome.RobotHome and not
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

    def apply_fault_actions(self, robot_obj: robot.Robot, fault_list):
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
            self._scheduler.schedule(self._total_steps)

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
        return "robot" in "".join(self._cells[y][x])

    def cell_within_faulty_robot_move_range(self, x, y):
        for faulty_robot_name, faulty_robot in self.sensor_faulty_bots.items():
            pos = faulty_robot.get_position()
            possible_next_positions = [(pos[0] + 1, pos[1]),
                                       (pos[0] - 1, pos[1]),
                                       (pos[0], pos[1] + 1),
                                       (pos[0], pos[1] - 1)]
            if (x, y) in possible_next_positions:
                return True
        return False



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
                robots_by_prio = reversed(sorted([robot_obj, blocking_robot], key=lambda robot2: robot2.get_prio()))
                is_horizontal = (blocking_robot.get_position()[0] - robot_obj.get_position()[0]) != 0
                self.move_robot_break_deadlock(robot_obj, robots_by_prio, is_horizontal)

    def attempt_resolve_deadlocks(self, robot_obj):
        robot_target = robot_obj.get_target()
        self.resolve_boxed_in_deadlock(robot_obj, robot_target.get_position()[0], robot_target.get_position()[1])

        keep_searching = True
        loop_found = False
        next_robot = self.get_robot_at(robot_target.get_position()[0], robot_target.get_position()[1])
        robots_searched = [robot_obj]
        if next_robot is not None:
            while keep_searching:
                robots_searched.append(next_robot)
                robot_target = next_robot.get_target()
                if robot_target is None:
                    break
                if robot_target.get_position() == next_robot.get_position():
                    break
                next_robot = self.get_robot_at(robot_target.get_position()[0], robot_target.get_position()[1])
                if next_robot is None:
                    break
                elif next_robot == robot_obj:
                    #print("CYCLIC DEADLOCK FOUND CONCERNING THIS ROBOT - OF SIZE %s" % (len(robots_searched)))
                    #print("Positions")
                    loop_found = True
                    keep_searching = False
                elif next_robot in robots_searched:
                    break
        if loop_found:
            robots_by_prio = reversed(sorted(robots_searched, key=lambda robot2: robot2.get_prio()))
            self.move_robot_break_deadlock(robot_obj, robots_by_prio)

    def get_robot_at(self, x, y):
        cell = self._cells[y][x]
        robot_name = None
        for obj_name in cell:
            if "robot" in obj_name:
                robot_name = obj_name
        if robot_name is None:
            return None
        return self._robots[robot_name]

    def move_robot_break_deadlock(self, this_robot, robots, prioritise_vertical=False):
        for robo in robots:
            if robo.get_wait_steps() != 0:
                continue
            if robo.get_target() is None:
                continue

            p = robo.get_position()

            vertical = [(p[0], p[1] + 1),
                        (p[0], p[1] - 1)]

            horizontal = [(p[0] + 1, p[1]),
                          (p[0] - 1, p[1])]

            offsets = horizontal + vertical

            if prioritise_vertical:
                offsets = vertical + horizontal

            for off in offsets:
                if not self.is_within_grid(off[0], off[1]):
                    continue
                if not self.cell_is_full(off[0], off[1]):
                    self.update_robot_position(robo.get_name(), off[0], off[1])
                    robo.set_movement_path(self.compute_robot_astar_path(robo))
                    if robo != this_robot:
                        robo.add_wait_steps(1)
                    return True
        return False

    def move_robots_away_from(self, x, y, robots):
        for robo in robots:
            if robo.get_wait_steps() != 0:
                continue
            x_change = robo.get_position()[0] - x
            y_change = robo.get_position()[1] - y
            new_x = robo.get_position()[0] + x_change
            new_y = robo.get_position()[1] + y_change
            if not self.is_within_grid(new_x, new_y):
                continue
            if not self.cell_is_full(new_x, new_y):
                if robo.get_target() is not None:
                    self.update_robot_position(robo.get_name(), new_x, new_y)
                    robo.set_movement_path(self.compute_robot_astar_path(robo))
                    robo.add_wait_steps(1)
                    return True
        return False

    def resolve_boxed_in_deadlock(self, robot_obj, x, y):
        offsets = [(x + 1, y),
                   (x - 1, y),
                   (x, y + 1),
                   (x, y - 1)]
        blocking_robots = []
        for off in offsets:
            if not self.is_within_grid(off[0], off[1]):
                continue
            if self.cell_contains_robot(off[0], off[1]):
                found_robot = self.get_robot_at(off[0], off[1])
                if found_robot.get_target() is None:
                    return
                if found_robot is not None:
                    blocking_robots.append(found_robot)
            else:
                return

        if len(blocking_robots) == 0:
            message = "An inaccessible target was located at (%s,%s)" % (x, y)
            raise customexceptions.SimulationError(message)

        #print("BOXED IN TARGET DETECTED")
        robots_by_prio = reversed(sorted(blocking_robots, key=lambda robot2: robot2.get_prio()))
        result = self.move_robots_away_from(x, y, robots_by_prio)

    def compute_robot_astar_path(self, robot_obj):
        robot_x, robot_y = robot_obj.get_position()
        target_x, target_y = robot_obj.get_target().get_position()

        @dataclass(order=True)
        class PrioNode:
            x: int = field(compare=False)
            y: int = field(compare=False)
            f_score: int

            def __eq__(self, other):
                return self.x == other.x and self.y == other.y
        g_scores = {}
        f_scores = {}
        came_from = {}
        search_frontier = []
        f_scores[(robot_x, robot_y)] = utils.taxicab_dist(robot_x, robot_y, target_x, target_y)
        g_scores[(robot_x, robot_y)] = 0
        heapq.heappush(search_frontier,
                       PrioNode(robot_x, robot_y, f_scores[(robot_x, robot_y)]))
        while len(search_frontier) != 0:
            current = heapq.heappop(search_frontier)
            if current.x == target_x and current.y == target_y:
                # We don't need the first element of the path, that's where we already are
                return utils.reconstruct_astar_path(came_from, (current.x, current.y))[1:]

            offsets = [(current.x + 1, current.y),
                       (current.x - 1, current.y),
                       (current.x, current.y + 1),
                       (current.x, current.y - 1)]

            # Search each neighbour
            for n in offsets:
                if not self.is_within_grid(n[0], n[1]):
                    continue
                neigh_tup = (n[0], n[1])
                cur_tup = (current.x, current.y)
                if not self.cell_contains_robot(n[0], n[1]):
                    possible_g_score = g_scores[cur_tup] + 1
                    if neigh_tup in g_scores.keys():
                        if not possible_g_score < g_scores[neigh_tup]:
                            continue
                    came_from[neigh_tup] = cur_tup
                    g_scores[neigh_tup] = possible_g_score
                    f_scores[neigh_tup] = possible_g_score + utils.taxicab_dist(current.x, current.y+1,
                                                                                target_x, target_y)
                    if PrioNode(n[0], n[1], 0) not in search_frontier:
                        heapq.heappush(search_frontier,
                                       PrioNode(n[0], n[1], f_scores[neigh_tup]))
        return []

    def transmit_initial_warehouse_layout(self):
        udptransmit.transmit_warehouse_size(self._width, self._height)
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

        f = open(filename, "r")
        for line_raw in f:
            lines.append(line_raw.strip())
        f.close()

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
                    new_robot = robot.Robot(new_robot_name, col_ctr, row_ctr, self._robot_max_inventory,
                                            self._robot_fault_rates)
                    self._robots[new_robot_name] = new_robot

                    new_home_name = "home%s" % robot_name_ctr
                    new_home = robothome.RobotHome(new_home_name, new_robot_name, col_ctr, row_ctr)
                    self._homes[new_home_name] = new_home

                    cells_copy[row_ctr].append([new_robot_name, new_home_name])
                    robot_name_ctr = robot_name_ctr + 1
                elif char == "S":
                    new_shelf_name = "shelf%s" % shelf_name_ctr
                    possible_item_name = "item%s" % shelf_name_ctr

                    if possible_item_name in self._items.keys():
                        new_shelf = shelf.Shelf(col_ctr, row_ctr, new_shelf_name, self._items[possible_item_name])
                    else:
                        new_shelf = shelf.Shelf(col_ctr, row_ctr, new_shelf_name)
                    self._shelves[new_shelf_name] = new_shelf

                    cells_copy[row_ctr].append([new_shelf_name])
                    shelf_name_ctr = shelf_name_ctr + 1
                elif char == "G":
                    new_goal_name = "goal%s" % goal_name_ctr
                    new_goal = orderstation.OrderStation(col_ctr, row_ctr, new_goal_name, self)
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
        udptransmit.transmit_warehouse_size(self._width, self._height)

    def generate_items(self, num_items):
        for i in range(num_items):
            item_name = "item%s" % i
            self._items[item_name] = item.Item(item_name, i)
            udptransmit.transmit_item_existence(item_name)

    def move_robot_next_path_spot(self, robot_obj):
        next_spot = robot_obj.get_movement_path()[0]
        self.update_robot_position(robot_obj.get_name(), next_spot[0], next_spot[1])
        robot_obj.get_movement_path().pop(0)

    def update_robot_position(self, robot_name, new_x, new_y):
        if self.cell_contains_robot(new_x, new_y):
            raise customexceptions.SimulationError("Two robots collided at (%s,%s)" % (new_x, new_y))

        robot_obj = self._robots[robot_name]
        old_x, old_y = robot_obj.get_position()
        robot_obj.set_position(new_x, new_y)
        self._cells[old_y][old_x].remove(robot_name)
        self._cells[new_y][new_x].append(robot_name)
        udptransmit.transmit_robot_position(robot_name, new_x, new_y)




