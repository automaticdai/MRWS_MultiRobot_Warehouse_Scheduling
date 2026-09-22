"""Order size must be decoupled from robot inventory size.

`Warehouse.__init__` capped order size at `robot_max_inventory`, so
`optimal_robots_required = ceil(len(items) / inventory_size)` in
`multi_robot_schedule_simple` was always 1. Every order was handed to a single
robot and the whole multi-robot branch -- item splitting, the block/flag
synchronisation protocol, the GA's multi-robot seeding -- was unreachable.
"""
import os
os.environ["ROBOTSIM_TRANSMIT"] = "False"  # never emit UDP during tests

import unittest

from mrws.engine.warehouse import Warehouse

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
WHOUSE = os.path.join(DATA_DIR, "whouse.txt")
INVENTORY_SIZE = 3


def build(mode, step_limit=2000):
    return Warehouse(WHOUSE, 10, INVENTORY_SIZE, mode, [0, 0, 0, 0], True, step_limit)


class TestOrderSize(unittest.TestCase):
    def test_orders_can_exceed_a_single_robot_inventory(self):
        warehouse = build("simple")
        order_manager = warehouse.get_order_manager()
        all_orders = order_manager._init_orders + order_manager._dynamic_orders
        largest = max(len(order_obj.get_items()) for order_obj in all_orders)
        self.assertGreater(
            largest, INVENTORY_SIZE,
            "no order needs more than one robot-load, so multi-robot can never engage",
        )

    def test_order_count_is_not_collapsed_by_larger_orders(self):
        # Bigger orders must not mean proportionally fewer of them: the order
        # budget is independent of how many items each order holds.
        warehouse = build("simple")
        order_manager = warehouse.get_order_manager()
        self.assertGreaterEqual(
            len(order_manager.get_init_orders()), 3,
            "initial order count collapsed when order size grew",
        )


class TestMultiRobotActuallyUsesMultipleRobots(unittest.TestCase):
    def _robots_per_order(self, mode):
        warehouse = build(mode)
        keep_stepping = True
        while keep_stepping:
            keep_stepping = not warehouse.step()
        return warehouse.get_scheduler().get_order_to_amount_of_robots_assigned()

    def test_multi_robot_mode_assigns_more_than_one_robot(self):
        counts = self._robots_per_order("multi-robot")
        self.assertTrue(counts, "no orders were scheduled at all")
        self.assertGreater(
            max(counts.values()), 1,
            "multi-robot mode still gave every order a single robot: %r" % counts,
        )


if __name__ == "__main__":
    unittest.main()
