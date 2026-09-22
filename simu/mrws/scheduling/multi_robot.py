import copy
import math
import random
from collections import deque

import pygad

from mrws.exceptions import SimulationError
from mrws.utils import taxicab_dist
from mrws.models.order import Order
from mrws.scheduling.ga_handler import GAHandler, fitness_func
from mrws.scheduling.ga_encoding import (
    genome_length, build_gene_space, decode_genome
)

# Search budget for one order. The assignment encoding makes every genome
# buildable, so the search is over a far smaller and better-behaved space than
# the old stop-list encoding needed, and a modest budget suffices. This runs
# once per order per reschedule, so it is also kept cheap on purpose.
GA_NUM_GENERATIONS = 25
GA_POPULATION_SIZE = 60
GA_PARENTS_MATING = 12


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


def emit_load_schedule(self, assignment, goal_name):
    """Write the block/flag schedule for an ordered list of (robot, item names).

    Load j hands over only once load j-1 has, so the order station receives
    items in decreasing dependency order. Both multi-robot schedulers build
    their schedules through here, so a genetic assignment is emitted as exactly
    the same structure the heuristic one produces.
    """
    for load_index, (robot_name, item_names) in enumerate(assignment):
        for item_name in item_names:
            self.add_to_schedule(robot_name, self._item_to_shelf_mapping[item_name][0])

        if load_index >= 1:
            self.add_to_schedule(robot_name, "block|flag%s" % self._mr_flag_ctr)
            self._mr_flag_ctr += 1

        if load_index != len(assignment) - 1:
            self.add_to_schedule(robot_name, "%s|flag%s" % (goal_name, self._mr_flag_ctr))
        else:
            self.add_to_schedule(robot_name, goal_name)


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
                # Every robot still working this order dumps its partial load,
                # because the faulted robot's items can no longer be sequenced
                # against theirs.
                for robot_name in non_faulted_bots:
                    robot_obj = self._robots[robot_name]
                    robot_obj.gone_home_to_clear_inv = True
                    robot_obj.set_target(self._homes[self.get_home_name_for_robot_name(robot_obj.get_name())])
                    self._schedule[robot_obj.get_name()] = deque()

                # One replacement order per faulted order, not one per healthy
                # robot: queueing the same order for removal twice popped the
                # same assignment key twice and raised KeyError.
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

            self.emit_load_schedule(list(zip(robot_name_list, item_names_split_by_bots)),
                                    free_goal_obj.get_name())
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

        assignment, goal_name = self.run_genetic_algorithm(order_obj, fault_tolerant_mode)

        # No goal or no robot free right now. The order stays in the backlog
        # and is retried on the next schedule call.
        if assignment is None:
            continue

        # A robot may hold several loads of the order, so record each of them
        # once: the rest of the scheduler treats this list as the set of robots
        # working the order.
        robot_names = list(dict.fromkeys(robot_name for robot_name, _ in assignment))
        self._order_robots_assignment[order_obj.get_id()] = robot_names
        self._order_goal_assignment[order_obj.get_id()] = goal_name
        for robot_name in robot_names:
            self._assign_robot_to_order(robot_name, order_obj.get_id())
            self._robots[robot_name].set_assigned_order(order_obj.get_id())
            self._robots[robot_name].set_prio(order_obj.get_prio())

        named_loads = [(robot_name, [item.get_name() for item in load])
                       for robot_name, load in assignment]
        self.emit_load_schedule(named_loads, goal_name)

        self._order_to_amount_robots_assigned[order_obj.get_id()] = len(robot_names)
        # Every genome is buildable, so the search no longer needs retries and
        # always succeeds on its first attempt.
        self._ga_attempts[0] += 1
        orders_to_move.append(order_obj)

    for ordr in orders_to_move:
        self._orders_backlog.remove(ordr)
        self._orders_active.append(ordr)
    return orders_to_move


def run_genetic_algorithm(self, order_obj, fault_tolerant_mode):
    """Search for the robot assignment that delivers this order soonest.

    Returns (assignment, goal_name), where assignment is an ordered list of
    (robot_name, [Item, ...]) loads, or (None, None) when the order cannot be
    scheduled right now.
    """
    goal_obj = self.find_goal_for_order(order_obj)
    if goal_obj is None:
        return None, None

    free_robots = self.find_free_robots(fault_tolerant_mode)
    if not free_robots:
        return None, None

    items_sorted = list(reversed(sorted(order_obj.get_items(),
                                        key=lambda itm: itm.get_dependency())))
    for item in items_sorted:
        if item.get_name() not in self._item_to_shelf_mapping:
            raise SimulationError("Scheduling impossible, no shelf exists for item %s"
                                  % item.get_name())

    # However few robots are free, the order can still be scheduled: a single
    # robot can carry it over several trips. How many to actually use is the
    # search's decision, not a precondition.
    robot_names = [robot.get_name() for robot in free_robots]

    self._update_robot_positions()

    # How far each robot's home is from this goal: a robot blocked on another
    # robot's flag is sent home, so this is what it has to travel back.
    goal_x, goal_y = goal_obj.get_position()
    home_distance_from_goal = {}
    for robot_name in robot_names:
        home_obj = self._homes[self.get_home_name_for_robot_name(robot_name)]
        home_x, home_y = home_obj.get_position()
        home_distance_from_goal[robot_name] = taxicab_dist(home_x, home_y, goal_x, goal_y)

    handler = GAHandler.get_instance()
    handler.configure(items_sorted, robot_names, goal_obj.get_name(),
                      self._ROBOT_INVENTORY_SIZE, self._all_positions,
                      self._item_to_shelf_mapping, home_distance_from_goal)

    num_genes = genome_length(len(items_sorted))
    ga_instance = pygad.GA(num_generations=GA_NUM_GENERATIONS,
                           num_genes=num_genes,
                           gene_type=int,
                           gene_space=build_gene_space(len(items_sorted), len(robot_names)),
                           sol_per_pop=GA_POPULATION_SIZE,
                           num_parents_mating=GA_PARENTS_MATING,
                           # Set explicitly: a small order has few genes, and
                           # pygad's default 10% rounds down to zero of them.
                           mutation_num_genes=max(1, num_genes // 10),
                           initial_population=seed_population(len(items_sorted),
                                                              len(robot_names)),
                           fitness_func=fitness_func)
    ga_instance.run()

    solution, _solution_fitness, _solution_idx = ga_instance.best_solution()
    assignment = decode_genome(solution, items_sorted,
                               self._ROBOT_INVENTORY_SIZE, robot_names)
    return assignment, goal_obj.get_name()


def seed_population(num_items, num_robots):
    """Starting population: evenly-cut splits at each load count, tried both as
    one robot making every trip and as a different robot per load, then random
    genomes to fill out the rest.

    Those two arrangements are the opposite ends of the trade the search exists
    to make -- cheap coordination against parallel collection -- so starting it
    from both means the budget goes on refining the choice rather than on
    stumbling across it.
    """
    population = []

    for load_count in range(1, min(num_items, 6) + 1):
        cut_genes = [0] * num_items
        for load_index in range(1, load_count):
            cut_genes[(load_index * num_items) // load_count] = 1

        for robot_index in range(min(num_robots, 3)):
            population.append(cut_genes + [robot_index] * num_items)

        for offset in range(min(num_robots, 2)):
            population.append(cut_genes
                              + [(index + offset) % num_robots for index in range(num_items)])

    while len(population) < GA_POPULATION_SIZE:
        population.append([random.randint(0, 1) for _ in range(num_items)]
                          + [random.randint(0, num_robots - 1) for _ in range(num_items)])

    return population[:GA_POPULATION_SIZE]
