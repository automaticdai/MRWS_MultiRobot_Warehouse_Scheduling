"""Fault reassignment must produce one replacement order per faulted order.

`reassign_orders_if_faulted` called `generate_order_to_complete_fault(order_id)`
once per *healthy* robot on a multi-robot order, so the same order object was
queued for removal N times. The removal loop then popped the same key twice and
raised KeyError. This was unreachable while orders only ever had one robot.
"""
import os
os.environ["ROBOTSIM_TRANSMIT"] = "False"  # never emit UDP during tests

import unittest

from mrws.engine.warehouse import Warehouse

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")
WHOUSE = os.path.join(DATA_DIR, "whouse.txt")


class TestMultiRobotFaultReassignment(unittest.TestCase):
    def _scheduler_with_shared_order(self, healthy_robots):
        """An order assigned to one faulted robot plus `healthy_robots` others."""
        warehouse = Warehouse(WHOUSE, 10, 3, "multi-robot", [0, 0, 0, 0], True, 2000)
        scheduler = warehouse.get_scheduler()

        order_obj = scheduler._orders_active[0]
        order_id = order_obj.get_id()
        assigned = ["robot%s" % n for n in range(healthy_robots + 1)]

        scheduler._order_robots_assignment = {order_id: assigned}
        scheduler._order_goal_assignment = {order_id: "goal0"}
        scheduler._orders_active = [order_obj]
        scheduler._orders_backlog = []
        warehouse._robots[assigned[0]].battery_faulted_critical = True

        return scheduler, order_obj

    def test_two_healthy_robots_produce_one_replacement_order(self):
        scheduler, order_obj = self._scheduler_with_shared_order(healthy_robots=2)

        scheduler.reassign_orders_if_faulted()

        self.assertEqual(
            len(scheduler._orders_backlog), 1,
            "expected exactly one replacement order, got %r" % scheduler._orders_backlog,
        )
        self.assertNotIn(order_obj, scheduler._orders_active)

    def test_healthy_robots_are_sent_home_to_clear_inventory(self):
        scheduler, _ = self._scheduler_with_shared_order(healthy_robots=2)

        scheduler.reassign_orders_if_faulted()

        for robot_name in ("robot1", "robot2"):
            self.assertTrue(
                scheduler._robots[robot_name].gone_home_to_clear_inv,
                "%s should have been sent home to dump its partial load" % robot_name,
            )

    def test_order_is_replaced_even_with_no_healthy_robots(self):
        # Every robot on the order faulted: the order still needs redoing.
        scheduler, order_obj = self._scheduler_with_shared_order(healthy_robots=1)
        scheduler._robots["robot1"].battery_faulted_critical = True

        scheduler.reassign_orders_if_faulted()

        self.assertEqual(len(scheduler._orders_backlog), 1)
        self.assertNotIn(order_obj, scheduler._orders_active)


if __name__ == "__main__":
    unittest.main()
