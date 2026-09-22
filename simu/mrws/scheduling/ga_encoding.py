"""Genome encoding for the genetic multi-robot scheduler.

A genome describes an *assignment*, not a list of stops:

    genome[0:n]    cut bits    -- does item i open a new robot load?
    genome[n:2n]   robot picks -- which robot carries load j?

where n is the number of items in the order and the items are always in
dependency-descending order.

Why an assignment and not a stop list: a single order station accepts items in
decreasing dependency order only, so the concatenation of every delivery, in
delivery order, has to be exactly the dependency-sorted item list. Each load
must therefore be a *contiguous* block of that list, and the loads must be
delivered in block order. Encoding the stops directly let a genome express
schedules that break this, so most of the population was infeasible and the
fitness function could only reject it -- leaving the search no gradient.

Here those constraints hold by construction, so every genome is a schedule that
can actually be built. What is left to search over is the part that is genuinely
open: how finely to cut the order, and which robot takes each load. A robot may
take several loads, which is what lets the search choose between one robot
making several trips and several robots each making one -- the real trade in
this warehouse, because parallel collection has to be paid for with handovers.
"""

# How much a robot's time counts against the makespan when scoring a schedule.
# At 0 the search only sees this one order and hands it most of the fleet; the
# cost of that falls on every other order, which this term is what accounts for.
#
# The value is empirical: swept against simulated step counts on whouse,
# whouse20x20 and wt15x15r5, then checked on three warehouses held out of the
# sweep. At 2.0 the genetic scheduler beats the heuristic one on four of those
# six and loses slightly on two, while still using ~2 robots per order rather
# than collapsing back to one.
ROBOT_TIME_WEIGHT = 2.0


def genome_length(num_items):
    """Genes needed for an order of `num_items` items."""
    return 2 * num_items


def build_gene_space(num_items, num_robots):
    """Per-gene value spaces, in the layout described above."""
    cut_space = [[0, 1] for _ in range(num_items)]
    robot_space = [list(range(num_robots)) for _ in range(num_items)]
    return cut_space + robot_space


def split_items_into_loads(cut_genes, items_sorted, inventory_size):
    """Cut dependency-sorted items into contiguous loads of at most
    `inventory_size`, opening a new load on a cut bit or when the current one is
    full.

    Every cut bit pattern is valid, so this never rejects a genome.
    """
    if not items_sorted:
        return []

    loads = []
    current = []
    for index, item in enumerate(items_sorted):
        if current and (len(current) == inventory_size or cut_genes[index] == 1):
            loads.append(current)
            current = []
        current.append(item)
    loads.append(current)

    return loads


def select_robots(robot_genes, num_loads, robot_names):
    """Pick the robot for each load.

    Repeats are allowed and meaningful: a robot taking consecutive loads is one
    robot making several trips, which needs no handover at all because its own
    delivery sets the flag the next load waits on.
    """
    fleet_size = len(robot_names)
    return [robot_names[int(robot_genes[load_index]) % fleet_size]
            for load_index in range(num_loads)]


def decode_genome(genome, items_sorted, inventory_size, robot_names):
    """Decode a genome into [(robot_name, [Item, ...]), ...] in delivery order.

    Returns None only when there is no robot to schedule onto at all.
    """
    if not robot_names:
        return None

    num_items = len(items_sorted)
    loads = split_items_into_loads(genome[:num_items], items_sorted, inventory_size)
    chosen = select_robots(genome[num_items:], len(loads), robot_names)
    return list(zip(chosen, loads))


def evaluate_schedule(assignment, shelf_for_item, distance_between, goal_name,
                      wait_cost_for_robot=None):
    """Returns (makespan, robot_time) for an assignment.

    `makespan` is the step the last load is delivered on. `robot_time` is the
    total time the schedule keeps robots occupied -- each robot is busy from the
    start until its own last delivery, so this is what the order costs the rest
    of the warehouse.

    Robots collect in parallel, but the block/flag chain serialises the
    deliveries: load j cannot be handed over before load j-1 has been, or the
    order station would receive items out of dependency order.

    What makes splitting an order a real trade rather than a free win is what
    happens to a robot that gets there early. A robot blocked on an unset flag
    is sent home (see Scheduler.parse_schedule_value), so it drifts away from
    the goal while it waits and has to travel back once released.
    `wait_cost_for_robot(robot_name)` is how far its home is from the goal,
    which bounds that drift. Waiting on your own previous delivery costs
    nothing, because it already set the flag -- which is why one robot making
    several trips can beat several robots making one each.
    """
    if wait_cost_for_robot is None:
        wait_cost_for_robot = lambda robot_name: 0

    robot_free_at = {}
    robot_location = {}
    delivered_at = 0
    previous_robot = None

    for robot_name, load in assignment:
        location = robot_location.get(robot_name, robot_name)

        travel = 0
        for item in load:
            shelf_name = shelf_for_item(item)
            travel += distance_between(location, shelf_name)
            location = shelf_name
        travel += distance_between(location, goal_name)

        arrival = robot_free_at.get(robot_name, 0) + travel

        if previous_robot is None:
            delivered_at = arrival                       # nothing to wait for
        elif previous_robot == robot_name:
            delivered_at = max(arrival, delivered_at)    # its own flag
        elif arrival >= delivered_at:
            delivered_at = arrival                       # flag already set
        else:
            drift = min(delivered_at - arrival, wait_cost_for_robot(robot_name))
            delivered_at = delivered_at + drift

        robot_free_at[robot_name] = delivered_at
        robot_location[robot_name] = goal_name
        previous_robot = robot_name

    return delivered_at, sum(robot_free_at.values())


def schedule_makespan(assignment, shelf_for_item, distance_between, goal_name,
                      wait_cost_for_robot=None):
    """Steps until the last load of this order is delivered."""
    return evaluate_schedule(assignment, shelf_for_item, distance_between,
                             goal_name, wait_cost_for_robot)[0]


def schedule_cost(assignment, shelf_for_item, distance_between, goal_name,
                  wait_cost_for_robot=None, robot_time_weight=ROBOT_TIME_WEIGHT):
    """What this schedule costs: how long the order takes, plus what holding
    those robots denies everything else.

    Makespan on its own says an extra robot is nearly free, because a robot
    collecting in parallel delays nobody. Scored that way the search hands each
    order most of the fleet, which is locally optimal and globally poor: the
    warehouse normally has more orders than robots, so orders queue behind the
    robots a greedy one is holding. Weighing robot-time alongside makespan is
    what makes the search trade latency on this order against throughput across
    all of them.
    """
    makespan, robot_time = evaluate_schedule(
        assignment, shelf_for_item, distance_between, goal_name, wait_cost_for_robot)
    return makespan + robot_time_weight * robot_time
