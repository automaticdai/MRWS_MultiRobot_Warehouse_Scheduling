"""Bridges the Scheduler and pygad for the genetic multi-robot scheduler.

pygad hands the fitness function nothing but a genome, so the context it needs
to score one -- the order's items, the free robots and where everything is --
lives on this singleton for the duration of a single search.

The genome layout and the decoding rules are in `ga_encoding`.
"""

from mrws.utils import taxicab_dist
from mrws.scheduling.ga_encoding import decode_genome, schedule_cost

# Stand-in for an unscorable genome. run_genetic_algorithm rules this case out
# before searching, so it should never be reached; it exists so a fitness
# evaluation can never take the whole simulation down.
UNSCHEDULABLE_FITNESS = -1e9


def fitness_func(ga_instance, solution, solution_idx):
    """Negated schedule cost, so cheaper schedules score higher.

    Every genome decodes into a schedule that can actually be built, so there is
    nothing to penalise and the objective is one continuous quantity the search
    can descend. The previous encoding spent most of its evaluations returning
    flat penalties for genomes describing schedules that were not buildable at
    all, which left selection nothing to distinguish them by.
    """
    handler = GAHandler.get_instance()
    assignment = handler.decode(solution)
    if assignment is None:
        return UNSCHEDULABLE_FITNESS
    return -float(handler.cost(assignment))


class GAHandler:
    instance = None

    @staticmethod
    def get_instance():
        if GAHandler.instance is None:
            GAHandler.instance = GAHandler()
        return GAHandler.instance

    def __init__(self):
        self._items_sorted = []
        self._robot_names = []
        self._goal_name = None
        self._max_inventory = 1
        self._positions = {}
        self._item_to_shelf_mapping = {}
        self._home_distance_from_goal = {}

    def configure(self, items_sorted, robot_names, goal_name, max_inventory,
                  positions, item_to_shelf_mapping, home_distance_from_goal):
        """Set the context for one search. Called before each pygad run.

        `home_distance_from_goal` maps a robot name to how far its home is from
        this order's goal, which is what a blocked robot has to travel back.
        """
        self._items_sorted = items_sorted
        self._robot_names = robot_names
        self._goal_name = goal_name
        self._max_inventory = max_inventory
        self._positions = positions
        self._item_to_shelf_mapping = item_to_shelf_mapping
        self._home_distance_from_goal = home_distance_from_goal

    def wait_cost_for_robot(self, robot_name):
        return self._home_distance_from_goal.get(robot_name, 0)

    def get_max_inventory(self):
        return self._max_inventory

    def shelf_for_item(self, item):
        return self._item_to_shelf_mapping[item.get_name()][0]

    def get_distance_between(self, name1, name2):
        if name1 == name2:
            return 0
        x1, y1 = self._positions[name1]
        x2, y2 = self._positions[name2]
        return taxicab_dist(x1, y1, x2, y2)

    def decode(self, genome):
        return decode_genome(genome, self._items_sorted,
                             self._max_inventory, self._robot_names)

    def cost(self, assignment):
        return schedule_cost(assignment, self.shelf_for_item,
                             self.get_distance_between, self._goal_name,
                             self.wait_cost_for_robot)
