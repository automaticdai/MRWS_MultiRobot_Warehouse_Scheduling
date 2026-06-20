from collections import deque

def taxicab_dist(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)


def robot_prio_sort_key(robot):
    """Sort key for ordering robots by priority.

    A robot's priority is None until it is assigned to an order (and is reset
    to None when its target is cleared via set_target(None)). Sorting robots
    directly by get_prio() therefore raises TypeError as soon as an unassigned
    robot is involved. Treat None as the lowest priority so the sort is total.
    """
    prio = robot.get_prio()
    return prio if prio is not None else -1


def reconstruct_astar_path(came_from: dict, current):
    total_path = deque([current])
    while current in came_from.keys():
        current = came_from[current]
        total_path.appendleft(current)
    return total_path
