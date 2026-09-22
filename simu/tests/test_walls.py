"""Walls (`W` cells) must be real obstacles.

`W` is parsed into `Warehouse._cells`, rendered by `print_layout_simple`, and
accepted by `validate_warehouse.py`, but nothing in the movement stack ever
consulted it: `compute_astar_path` only treated robots as obstacles, so paths
were routed straight through walls.
"""
import os
os.environ["ROBOTSIM_TRANSMIT"] = "False"  # never emit UDP during tests

import tempfile
import unittest

from mrws.engine.pathfinding import compute_astar_path
from mrws.engine.warehouse import Warehouse


def write_warehouse(layout):
    """Write a layout string to a temp file and return its path."""
    handle = tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False)
    handle.write(layout.strip("\n"))
    handle.close()
    return handle.name


# Robot at (0,2), shelf at (2,1), goal at (4,0). The direct route from the
# robot to the shelf runs through the wall band at y=2; a detour exists over
# the top of the grid.
DETOUR_LAYOUT = """
XXXXX
RWWWX
XWSXX
XXXXG
"""

# The shelf at (2,2) is sealed in on all four sides.
SEALED_LAYOUT = """
XXXXX
XWWWX
XWSWX
XWWWX
XXXXX
"""


class TestAstarRespectsWalls(unittest.TestCase):
    def _walls(self, warehouse):
        return {
            (x, y)
            for y, row in enumerate(warehouse._cells)
            for x, cell in enumerate(row)
            if cell == ["wall"]
        }

    def test_path_detours_around_walls(self):
        path = compute_astar_path(
            5, 4, {}, (0, 2), (2, 1),
            blocked_cells={(1, 2), (2, 2), (3, 2), (1, 1)},
        )
        self.assertTrue(path, "expected a path around the wall band")
        self.assertEqual(
            [cell for cell in path if cell in {(1, 2), (2, 2), (3, 2), (1, 1)}],
            [],
            "path must not cross any wall cell",
        )

    def test_no_path_to_sealed_target(self):
        walls = {(1, 1), (2, 1), (3, 1), (1, 2), (3, 2), (1, 3), (2, 3), (3, 3)}
        path = compute_astar_path(5, 5, {}, (0, 0), (2, 2), blocked_cells=walls)
        self.assertEqual(list(path), [], "a sealed target must be unreachable")

    def test_warehouse_reports_wall_cells(self):
        warehouse = Warehouse(write_warehouse(DETOUR_LAYOUT), 1, 3, "simple",
                              [0, 0, 0, 0], True, 200)
        self.assertEqual(
            warehouse.get_wall_cells(), {(1, 2), (2, 2), (3, 2), (1, 1)}
        )

    def test_wall_cell_counts_as_full(self):
        warehouse = Warehouse(write_warehouse(DETOUR_LAYOUT), 1, 3, "simple",
                              [0, 0, 0, 0], True, 200)
        self.assertTrue(warehouse.cell_is_full(1, 2), "a wall cell is not enterable")
        self.assertFalse(warehouse.cell_is_full(0, 0), "an empty floor cell is enterable")


class TestRobotsNeverEnterWalls(unittest.TestCase):
    def test_robot_stays_out_of_walls_for_a_whole_run(self):
        path = write_warehouse(DETOUR_LAYOUT)
        warehouse = Warehouse(path, 1, 3, "simple", [0, 0, 0, 0], True, 300)
        walls = {(1, 2), (2, 2), (3, 2), (1, 1)}

        keep_stepping = True
        while keep_stepping:
            keep_stepping = not warehouse.step()
            for robot_obj in warehouse._robots.values():
                self.assertNotIn(
                    robot_obj.get_position(), walls,
                    "robot %s walked into a wall" % robot_obj.get_name(),
                )


if __name__ == "__main__":
    unittest.main()
