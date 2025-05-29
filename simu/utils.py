def taxicab_dist(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)


def reconstruct_astar_path(came_from: dict, current):
    total_path = [current]
    while current in came_from.keys():
        current = came_from[current]
        total_path.insert(0, current)
    return total_path
