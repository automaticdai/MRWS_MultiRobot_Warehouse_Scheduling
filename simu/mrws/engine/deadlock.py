from mrws.exceptions import SimulationError
from mrws.utils import robot_prio_sort_key


def attempt_resolve_deadlocks(self, robot_obj):
    robot_target = robot_obj.get_target()
    self.resolve_boxed_in_deadlock(robot_obj, robot_target.get_position()[0], robot_target.get_position()[1])

    keep_searching = True
    loop_found = False
    next_robot = self.get_robot_at(robot_target.get_position()[0], robot_target.get_position()[1])
    robots_searched = [robot_obj]
    robots_searched_set = {id(robot_obj)}
    if next_robot is not None:
        while keep_searching:
            robots_searched.append(next_robot)
            robots_searched_set.add(id(next_robot))
            robot_target = next_robot.get_target()
            if robot_target is None:
                break
            if robot_target.get_position() == next_robot.get_position():
                break
            next_robot = self.get_robot_at(robot_target.get_position()[0], robot_target.get_position()[1])
            if next_robot is None:
                break
            elif next_robot == robot_obj:
                loop_found = True
                keep_searching = False
            elif id(next_robot) in robots_searched_set:
                break
    if loop_found:
        robots_by_prio = reversed(sorted(robots_searched, key=robot_prio_sort_key))
        self.move_robot_break_deadlock(robot_obj, robots_by_prio)

def get_robot_at(self, x, y):
    robot_name = self._position_to_robot.get((x, y))
    if robot_name is None:
        return None
    return self._robots[robot_name]

def move_robot_break_deadlock(self, this_robot, robots, prioritise_vertical=False):
    for robo in robots:
        if robo.get_wait_steps() != 0:
            continue
        if robo.get_target() is None:
            continue

        p = robo.get_position()

        vertical = [(p[0], p[1] + 1),
                    (p[0], p[1] - 1)]

        horizontal = [(p[0] + 1, p[1]),
                      (p[0] - 1, p[1])]

        offsets = horizontal + vertical

        if prioritise_vertical:
            offsets = vertical + horizontal

        for off in offsets:
            if not self.is_within_grid(off[0], off[1]):
                continue
            if not self.cell_is_full(off[0], off[1]):
                self.update_robot_position(robo.get_name(), off[0], off[1])
                robo.set_movement_path(self.compute_robot_astar_path(robo))
                if robo != this_robot:
                    robo.add_wait_steps(1)
                return True
    return False

def move_robots_away_from(self, x, y, robots):
    for robo in robots:
        if robo.get_wait_steps() != 0:
            continue
        x_change = robo.get_position()[0] - x
        y_change = robo.get_position()[1] - y
        new_x = robo.get_position()[0] + x_change
        new_y = robo.get_position()[1] + y_change
        if not self.is_within_grid(new_x, new_y):
            continue
        if not self.cell_is_full(new_x, new_y):
            if robo.get_target() is not None:
                self.update_robot_position(robo.get_name(), new_x, new_y)
                robo.set_movement_path(self.compute_robot_astar_path(robo))
                robo.add_wait_steps(1)
                return True
    return False

def resolve_boxed_in_deadlock(self, robot_obj, x, y):
    offsets = [(x + 1, y),
               (x - 1, y),
               (x, y + 1),
               (x, y - 1)]
    blocking_robots = []
    for off in offsets:
        if not self.is_within_grid(off[0], off[1]):
            continue
        if self.cell_contains_robot(off[0], off[1]):
            found_robot = self.get_robot_at(off[0], off[1])
            if found_robot.get_target() is None:
                return
            if found_robot is not None:
                blocking_robots.append(found_robot)
        else:
            return

    if len(blocking_robots) == 0:
        message = "An inaccessible target was located at (%s,%s)" % (x, y)
        raise SimulationError(message)

    #print("BOXED IN TARGET DETECTED")
    robots_by_prio = reversed(sorted(blocking_robots, key=robot_prio_sort_key))
    result = self.move_robots_away_from(x, y, robots_by_prio)
