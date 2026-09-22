import heapq
from dataclasses import dataclass, field

from mrws.utils import taxicab_dist, reconstruct_astar_path

@dataclass(order=True)
class PrioNode:
    x: int = field(compare=False)
    y: int = field(compare=False)
    f_score: int


def compute_astar_path(width, height, position_to_robot, start_pos, target_pos,
                       blocked_cells=frozenset()):
    """A* over the grid. Robots and `blocked_cells` (walls) are impassable."""
    robot_x, robot_y = start_pos
    target_x, target_y = target_pos

    g_scores = {}
    came_from = {}
    search_frontier = []
    start_f = taxicab_dist(robot_x, robot_y, target_x, target_y)
    g_scores[(robot_x, robot_y)] = 0
    heapq.heappush(search_frontier,
                   PrioNode(robot_x, robot_y, start_f))
    while len(search_frontier) != 0:
        current = heapq.heappop(search_frontier)
        cur_tup = (current.x, current.y)

        if current.x == target_x and current.y == target_y:
            path = reconstruct_astar_path(came_from, cur_tup)
            # Remove first element (current position); path is already a deque
            path.popleft()
            return path

        offsets = [(current.x + 1, current.y),
                   (current.x - 1, current.y),
                   (current.x, current.y + 1),
                   (current.x, current.y - 1)]

        for n in offsets:
            if not (0 <= n[0] <= width - 1 and 0 <= n[1] <= height - 1):
                continue
            neigh_tup = (n[0], n[1])
            if neigh_tup not in position_to_robot and neigh_tup not in blocked_cells:
                possible_g_score = g_scores[cur_tup] + 1
                if neigh_tup in g_scores:
                    if possible_g_score >= g_scores[neigh_tup]:
                        continue
                came_from[neigh_tup] = cur_tup
                g_scores[neigh_tup] = possible_g_score
                f_score = possible_g_score + taxicab_dist(n[0], n[1],
                                                          target_x, target_y)
                heapq.heappush(search_frontier,
                               PrioNode(n[0], n[1], f_score))
    return []
