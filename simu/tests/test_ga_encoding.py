"""The genetic encoder must make every genome a buildable schedule.

The old encoding was a literal list of stops, so most genomes described
schedules that could not be executed and the fitness function had to reject
them. This encoding describes an *assignment* instead -- how the dependency
sorted items are cut into loads, and which robot takes each load -- so a genome
can never be infeasible, which is what gives the search a gradient to follow.
"""
import os
os.environ["ROBOTSIM_TRANSMIT"] = "False"  # never emit UDP during tests

import random
import unittest

from mrws.models.item import Item
from mrws.scheduling.ga_encoding import (
    genome_length,
    build_gene_space,
    split_items_into_loads,
    select_robots,
    decode_genome,
    schedule_makespan,
)

INV = 3


def items(count):
    """Items in dependency-descending order, as the scheduler sorts them."""
    return [Item("item%d" % n, count - n) for n in range(count)]


def robots(count):
    return ["robot%d" % n for n in range(count)]


class TestGenomeShape(unittest.TestCase):
    def test_genome_has_two_genes_per_item(self):
        self.assertEqual(genome_length(9), 18)

    def test_gene_space_is_cut_bits_then_robot_indices(self):
        space = build_gene_space(num_items=4, num_robots=3)
        self.assertEqual(len(space), 8)
        self.assertEqual(space[:4], [[0, 1]] * 4)
        self.assertEqual(space[4:], [[0, 1, 2]] * 4)


class TestSplittingItemsIntoLoads(unittest.TestCase):
    def test_a_cut_bit_opens_a_new_load(self):
        loads = split_items_into_loads([0, 0, 1, 0], items(4), INV)
        self.assertEqual([[i.get_name() for i in load] for load in loads],
                         [["item0", "item1"], ["item2", "item3"]])

    def test_a_full_load_opens_a_new_load_without_a_cut(self):
        loads = split_items_into_loads([0, 0, 0, 0], items(4), INV)
        self.assertEqual([len(load) for load in loads], [3, 1])

    def test_loads_never_exceed_inventory_size(self):
        loads = split_items_into_loads([1] * 9, items(9), INV)
        for load in loads:
            self.assertLessEqual(len(load), INV)
            self.assertGreater(len(load), 0)

    def test_loads_are_contiguous_and_lose_nothing(self):
        source = items(9)
        loads = split_items_into_loads([0, 1, 0, 1, 1, 0, 0, 1, 0], source, INV)
        flattened = [item for load in loads for item in load]
        self.assertEqual([i.get_name() for i in flattened],
                         [i.get_name() for i in source])


class TestSelectingRobots(unittest.TestCase):
    def test_gene_picks_the_named_robot(self):
        chosen = select_robots([3, 1, 0], num_loads=3, robot_names=robots(4))
        self.assertEqual(chosen, ["robot3", "robot1", "robot0"])

    def test_a_robot_may_take_several_loads(self):
        # One robot making three trips is a legal -- and often the best --
        # schedule, because consecutive loads by one robot need no handover.
        chosen = select_robots([2, 2, 2], num_loads=3, robot_names=robots(4))
        self.assertEqual(chosen, ["robot2", "robot2", "robot2"])

    def test_a_gene_out_of_range_wraps_onto_the_fleet(self):
        chosen = select_robots([7], num_loads=1, robot_names=robots(3))
        self.assertIn(chosen[0], robots(3))


class TestDecodeIsTotal(unittest.TestCase):
    def test_every_random_genome_decodes_to_a_valid_assignment(self):
        rng = random.Random(20260922)
        source = items(9)
        for fleet_size in (1, 2, 5):
            fleet = robots(fleet_size)
            for _ in range(300):
                genome = ([rng.randint(0, 1) for _ in range(9)]
                          + [rng.randint(0, fleet_size - 1) for _ in range(9)])
                assignment = decode_genome(genome, source, INV, fleet)
                self.assertIsNotNone(assignment, "decode must never reject a genome")

                delivered = [item for _, load in assignment for item in load]
                self.assertEqual([i.get_name() for i in delivered],
                                 [i.get_name() for i in source],
                                 "loads must stay in dependency order and lose nothing")
                for robot_name, load in assignment:
                    self.assertIn(robot_name, fleet)
                    self.assertGreater(len(load), 0)
                    self.assertLessEqual(len(load), INV)

    def test_a_single_robot_can_carry_any_order(self):
        assignment = decode_genome([0] * 18, items(9), INV, ["robot0"])
        self.assertEqual({name for name, _ in assignment}, {"robot0"})
        self.assertEqual(sum(len(load) for _, load in assignment), 9)

    def test_no_robots_is_the_only_rejection(self):
        self.assertIsNone(decode_genome([0] * 18, items(9), INV, []))


class TestMakespan(unittest.TestCase):
    """robot1 sits on both the goal and its own shelf; robot0 is far away."""

    positions = {"robot0": (0, 0), "robot1": (10, 0),
                 "shelfA": (100, 0), "shelfB": (10, 0), "goal0": (10, 0)}
    item_a = Item("a", 2)
    item_b = Item("b", 1)

    def distance(self, a, b):
        pa, pb = self.positions[a], self.positions[b]
        return abs(pa[0] - pb[0]) + abs(pa[1] - pb[1])

    def shelf_for(self, item):
        return {"a": "shelfA", "b": "shelfB"}[item.get_name()]

    def makespan(self, assignment, wait_cost=None):
        return schedule_makespan(
            assignment, self.shelf_for, self.distance, "goal0",
            None if wait_cost is None else (lambda robot_name: wait_cost))

    def test_deliveries_are_serialised_by_the_flag_chain(self):
        # robot1 is ready immediately but cannot deliver before robot0 has.
        # robot0's trip is 0->100->goal(10) = 190.
        self.assertEqual(
            self.makespan([("robot0", [self.item_a]), ("robot1", [self.item_b])]),
            190)

    def test_a_robot_blocked_on_another_robots_flag_pays_to_come_back(self):
        # robot1 arrives at 0 and waits until 190, so it is sent home and has to
        # travel back: its home is 25 away from the goal.
        self.assertEqual(
            self.makespan([("robot0", [self.item_a]), ("robot1", [self.item_b])],
                          wait_cost=25),
            215)

    def test_waiting_on_your_own_flag_is_free(self):
        # One robot doing both loads sets its own flag, so it is never sent
        # home -- this is why several trips by one robot can beat one trip each.
        one_robot = [("robot0", [self.item_a]), ("robot0", [self.item_b])]
        self.assertEqual(self.makespan(one_robot, wait_cost=25),
                         self.makespan(one_robot))

    def test_a_robot_arriving_after_the_flag_pays_nothing(self):
        # robot0 arrives at 190, long after robot1 delivered at 0, so it never
        # blocks and is never sent home.
        late_arrival = [("robot1", [self.item_b]), ("robot0", [self.item_a])]
        self.assertEqual(self.makespan(late_arrival, wait_cost=25), 190)

    def test_the_first_load_waits_for_nobody(self):
        self.assertEqual(self.makespan([("robot1", [self.item_b])], wait_cost=25), 0)

    def test_a_second_load_starts_from_the_goal_after_the_first(self):
        # robot0 delivers at 190, then runs goal->shelfB(0)->goal(0).
        self.assertEqual(
            self.makespan([("robot0", [self.item_a]), ("robot0", [self.item_b])]),
            190)

    def test_shorter_trips_score_a_shorter_makespan(self):
        near = self.makespan([("robot1", [self.item_a])])
        far = self.makespan([("robot0", [self.item_a])])
        self.assertLess(near, far)


if __name__ == "__main__":
    unittest.main()
