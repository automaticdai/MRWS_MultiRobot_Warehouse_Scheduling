"""Regression tests for the three critical crash bugs found in code review.

Run from the `simu/` directory:
    conda run -n MRWS python -m unittest tests.test_critical_fixes -v
"""
import os
os.environ["ROBOTSIM_TRANSMIT"] = "False"  # never emit UDP during tests

import unittest

from mrws.entities.robot import Robot


class TestMultiRobotItemSplit(unittest.TestCase):
    """Bug 1: items must be split across robots in chunks of inventory_size,
    one chunk per robot. The old loop never reset the per-robot counter, so it
    only ever produced two groups and overflowed robot 1 for orders needing 3+
    robots."""

    def test_split_7_items_3_robots_inv3(self):
        from mrws.scheduling.multi_robot import split_items_among_robots
        names = ["i0", "i1", "i2", "i3", "i4", "i5", "i6"]
        groups = split_items_among_robots(names, num_robots=3, inventory_size=3)
        self.assertEqual(groups, [["i0", "i1", "i2"], ["i3", "i4", "i5"], ["i6"]])

    def test_no_robot_exceeds_inventory(self):
        from mrws.scheduling.multi_robot import split_items_among_robots
        # 9 items, 3 robots, inv 3 -> exactly [3,3,3]; the old code put 6 on robot 1.
        names = [f"i{n}" for n in range(9)]
        groups = split_items_among_robots(names, num_robots=3, inventory_size=3)
        self.assertEqual(len(groups), 3)
        for g in groups:
            self.assertLessEqual(len(g), 3)
        # every item placed exactly once
        self.assertEqual(sum(groups, []), names)

    def test_two_robot_case_still_correct(self):
        # Regression guard: the one case the old code got right must stay right.
        from mrws.scheduling.multi_robot import split_items_among_robots
        names = ["i0", "i1", "i2", "i3", "i4", "i5"]
        groups = split_items_among_robots(names, num_robots=2, inventory_size=3)
        self.assertEqual(groups, [["i0", "i1", "i2"], ["i3", "i4", "i5"]])


class TestRobotPrioSortKey(unittest.TestCase):
    """Bug 2: sorting robots by get_prio() crashes when any prio is None
    (a robot's prio starts None and is reset to None by set_target(None))."""

    def _robot(self, name, prio=None):
        r = Robot(name, 0, 0, 3, [0, 0, 0, 0])
        if prio is not None:
            r.set_prio(prio)
        return r

    def test_sort_with_none_prio_does_not_crash(self):
        from mrws.utils import robot_prio_sort_key
        r_none = self._robot("robot0")          # prio None
        r5 = self._robot("robot1", prio=5)
        # Must not raise TypeError; None is treated as lowest priority.
        ordered = sorted([r5, r_none], key=robot_prio_sort_key)
        self.assertEqual([r.get_name() for r in ordered], ["robot0", "robot1"])

    def test_reversed_sort_puts_highest_first(self):
        from mrws.utils import robot_prio_sort_key
        r_none = self._robot("robot0")
        r2 = self._robot("robot1", prio=2)
        r5 = self._robot("robot2", prio=5)
        ordered = list(reversed(sorted([r_none, r2, r5], key=robot_prio_sort_key)))
        self.assertEqual([r.get_name() for r in ordered], ["robot2", "robot1", "robot0"])


class TestSensorEvasionMovesOnce(unittest.TestCase):
    """Bug 3: a robot fleeing a sensor-faulted neighbour must move at most one
    cell. The old loop had no break, so it called update_robot_position for
    every free neighbour, teleporting the robot multiple cells per step."""

    def _make_warehouse(self):
        from mrws.engine.warehouse import Warehouse

        class _SpyWarehouse(Warehouse):
            def __init__(self):
                self.move_calls = 0

            def update_robot_position(self, robot_name, new_x, new_y):
                self.move_calls += 1
                super().update_robot_position(robot_name, new_x, new_y)

        class _NoopScheduler:
            # decide_robot_action consults the scheduler after the evasion
            # branch; that interaction is irrelevant to what we measure here.
            def direct_robot(self, robot_obj):
                pass

        wh = _SpyWarehouse()
        wh._fault_tolerant_mode = True
        wh._width = 5
        wh._height = 5
        wh._cells = [[[] for _ in range(5)] for _ in range(5)]
        wh._robots = {}
        wh._position_to_robot = {}
        wh._faulty_blocked_cells = set()
        wh.sensor_faulty_bots = {}
        wh._scheduler = _NoopScheduler()
        return wh

    def _place_robot(self, wh, name, x, y):
        r = Robot(name, x, y, 3, [0, 0, 0, 0])
        wh._robots[name] = r
        wh._cells[y][x].append(name)
        wh._position_to_robot[(x, y)] = name
        return r

    def test_robot_in_faulty_range_moves_at_most_once(self):
        wh = self._make_warehouse()
        robot = self._place_robot(wh, "robot0", 2, 2)
        # Mark the robot's own cell as within a faulty robot's move range, so the
        # evasion branch triggers. All four neighbours are free.
        wh._faulty_blocked_cells.add((2, 2))

        wh.decide_robot_action(robot)

        self.assertLessEqual(wh.move_calls, 1)
        # And it actually relocated to a single adjacent cell.
        new_pos = robot.get_position()
        manhattan = abs(new_pos[0] - 2) + abs(new_pos[1] - 2)
        self.assertEqual(manhattan, 1)


if __name__ == "__main__":
    unittest.main()
