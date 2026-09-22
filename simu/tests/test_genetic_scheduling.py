"""The genetic scheduler must handle orders that need more than one robot.

Orders are sized at up to three robot-loads, so the genetic mode has to split an
order across robots. The old stop-list encoding could not: it failed every run
at nine items and 11/20 at six, because most genomes described schedules that
could not be executed and the fitness function could only reject them.

`ga_encoding` replaces it with an assignment encoding whose every genome is a
buildable schedule. See `tests/test_ga_encoding.py` for the encoding invariants.
"""
import os
os.environ["ROBOTSIM_TRANSMIT"] = "False"  # never emit UDP during tests

import unittest

from mrws.engine.warehouse import Warehouse

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
WHOUSE = os.path.join(DATA_DIR, "whouse.txt")
SMALL_WHOUSE = os.path.join(DATA_DIR, "whouse10x10.txt")


class TestGeneticSchedulerHandlesMultiRobotOrders(unittest.TestCase):
    RUNS = 5

    def test_genetic_mode_completes_full_simulations(self):
        failures = []
        for run in range(self.RUNS):
            try:
                warehouse = Warehouse(WHOUSE, 10, 3, "multi-robot-genetic",
                                      [0, 0, 0, 0], True, 4000)
                keep_stepping = True
                while keep_stepping:
                    keep_stepping = not warehouse.step()
            except Exception as err:
                failures.append("run %d: %s: %s" % (run, type(err).__name__, err))

        self.assertEqual(
            failures, [],
            "genetic mode failed %d/%d runs:\n  %s"
            % (len(failures), self.RUNS, "\n  ".join(failures)),
        )

    def test_genetic_mode_assigns_multiple_robots_to_a_large_order(self):
        warehouse = Warehouse(WHOUSE, 10, 3, "multi-robot-genetic",
                              [0, 0, 0, 0], True, 4000)
        assignments = warehouse.get_scheduler()._order_robots_assignment
        self.assertTrue(assignments, "no orders were scheduled")
        self.assertGreater(
            max(len(robots) for robots in assignments.values()), 1,
            "GA still put every order on a single robot: %r" % assignments,
        )

    def test_a_robot_is_never_given_two_loads_of_one_order(self):
        warehouse = Warehouse(WHOUSE, 10, 3, "multi-robot-genetic",
                              [0, 0, 0, 0], True, 4000)
        for order_id, robots in warehouse.get_scheduler()._order_robots_assignment.items():
            self.assertEqual(len(set(robots)), len(robots),
                             "order %s has a duplicate robot: %r" % (order_id, robots))


class TestGeneticSchedulerWithFewerGoalsThanOrders(unittest.TestCase):
    def test_orders_outnumbering_goals_do_not_crash(self):
        # whouse10x10 has 2 goals but more initial orders than that. The old
        # run_genetic_algorithm dereferenced find_goal_for_order() without a
        # None check and raised AttributeError before the simulation started.
        warehouse = Warehouse(SMALL_WHOUSE, 10, 3, "multi-robot-genetic",
                              [0, 0, 0, 0], True, 4000)
        keep_stepping = True
        while keep_stepping:
            keep_stepping = not warehouse.step()


if __name__ == "__main__":
    unittest.main()
