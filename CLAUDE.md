# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MRWS (MultiRobot Warehouse Simulator) is a Python simulator for robotic smart warehouses. It supports multiple scheduling/allocation algorithms and can visualize simulations via a Unity client.

## Running the Simulator

The codebase is organized as a Python package (`mrws/`) under `simu/`. CLI scripts live at `simu/` level and the working directory must be `simu/`:

```bash
cd simu
python main.py              # Run 1000 sims (default)
python main.py -n 1         # Run 1 simulation
python main.py -t           # Run with UDP transmission to Unity visualizer
python main.py -t -n 1      # Single sim with visualization
```

The conda environment is `MRWS`:
```bash
cd simu
conda run -n MRWS python main.py -n 1
```

The `-t` flag sets `ROBOTSIM_TRANSMIT=True` in the environment, which `mrws/io/udp.py` checks before sending any UDP packets. The `slow_for_transmit` parameter in `run_simulation()` adds 200ms delays between steps for visualization.

### Verifying All Scheduling Modes

```bash
cd simu
python -c "
import os; os.environ['ROBOTSIM_TRANSMIT'] = 'False'
from main import Simulation
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath('main.py')), 'data')
for mode in ['simple', 'simple-interrupt', 'multi-robot', 'multi-robot-genetic']:
    sim = Simulation(1, os.path.join(DATA_DIR, 'whouse.txt'), 10, 3, mode, [0,0,0,0], True, 1000)
    sim.run_simulation(True, False)
    print(f'{mode} OK')
"
```

### Tests

```bash
cd simu
conda run -n MRWS python -m unittest discover -s tests
```

`simu/tests/` covers the encoding invariants and past crash bugs. There is no linting or formatting tooling configured.

## Package Structure

```
simu/
  main.py                          # CLI entry point (Simulation class, batch experiments)
  generate_warehouse.py            # CLI tool for generating warehouse files
  validate_warehouse.py            # CLI tool for validating warehouse files
  data/                            # Warehouse files (.txt)
  mrws/                            # Main Python package
    __init__.py
    exceptions.py                  # SimulationError
    utils.py                       # taxicab_dist, reconstruct_astar_path

    models/
      item.py                      # Item (name + dependency)
      order.py                     # Order (items, priority, id)

    entities/
      inventory.py                 # InventoryEntity base class (LIFO stack)
      robot.py                     # Robot(InventoryEntity)
      shelf.py                     # Shelf
      order_station.py             # OrderStation(InventoryEntity)
      home.py                      # RobotHome

    scheduling/
      scheduler.py                 # Scheduler core: __init__, schedule(), helpers
      simple.py                    # simple_single_robot_schedule, single_interrupt_robot_schedule
      multi_robot.py               # multi_robot_schedule_*, run_genetic_algorithm, fault reassignment
      ga_encoding.py               # Genome layout, decoding, schedule cost model
      ga_handler.py                # GAHandler singleton, fitness_func

    engine/
      warehouse.py                 # Warehouse: init, step, parsing, movement, faults
      pathfinding.py               # PrioNode + compute_astar_path standalone function
      deadlock.py                  # Deadlock detection & resolution (attached to Warehouse)
      order_manager.py             # OrderManager

    io/
      udp.py                       # UDP transmit functions
      gui.py                       # PyQt6 GUI (MRWS GUI)
```

### Split File Pattern

`scheduler.py` and `warehouse.py` are split across multiple files using method attachment:

```python
# scheduling/simple.py defines functions taking `self` as first param
def simple_single_robot_schedule(self, fault_tolerant_mode, ...):
    ...

# scheduling/scheduler.py attaches them to the Scheduler class
class Scheduler:
    simple_single_robot_schedule = simple_single_robot_schedule
```

Similarly, `engine/deadlock.py` defines methods attached to `Warehouse`, and `engine/pathfinding.py` provides a standalone `compute_astar_path()` function called by `Warehouse.compute_robot_astar_path()`.

## Key Architecture

### Simulation Entry Point (`main.py`)

`Simulation` is the top-level runner. It takes: number of sims, warehouse file, number of items, robot inventory size, scheduling mode, fault rates (list of 4 floats), fault tolerance flag, and step limit. The warehouse file path must be absolute or relative to `simu/`; `DATA_DIR` points to `simu/data/`.

Helper functions (`run_completion_time_test`, `run_fault_test`, `run_simulation_performance_test`) run batch experiments and plot results with matplotlib.

### Core Simulation Loop

The simulation runs in discrete time steps managed by `Warehouse.step()`:
1. Each robot: apply faults, decide action (pathfind via A*, interact with target, or wait for scheduler), move
2. Possibly introduce a dynamic order (before the `dynamic_deadline` step)
3. Returns `True` when all orders are complete and past the dynamic deadline

### Coordinate System

Warehouse cell `(x,y)` is accessed via `self._cells[y][x]`. The file is parsed bottom-up (`reversed(lines)`), so `y=0` corresponds to the last line in the warehouse file. Each cell is a list of entity name strings (e.g., `["robot0", "home0"]`).

### Entity Naming and Interaction

Entities are named sequentially: `robot0`, `robot1`, `shelf0`, `shelf1`, `goal0`, `home0`, `item0`, etc. The `homeN` always corresponds to `robotN` (used by `get_home_name_for_robot_name`).

All interactable entities (`Shelf`, `OrderStation`, `RobotHome`) implement an `interact(obj)` method called when a robot arrives at their position. Shelves give items, order stations receive inventory and check for order completion, homes handle charging/clearing.

### Inventory and Item Dependencies

`InventoryEntity` is a base class (used by `Robot` and `OrderStation`) implementing a LIFO stack inventory. Items have a dependency number, and **items must be added in decreasing dependency order**. This constraint drives the scheduling logic — items are sorted by dependency (highest first) when building schedules.

### Scheduling Modes

Four scheduling algorithms in `Scheduler` (split across `scheduling/scheduler.py`, `simple.py`, `multi_robot.py`):
- `simple`: One robot per order, FIFO assignment
- `simple-interrupt`: One robot with priority-based preemption (can steal a lower-priority robot mid-task)
- `multi-robot`: Multiple robots per order when enough are free; falls back to single-robot
- `multi-robot-genetic`: Uses `pygad` GA to optimize multi-robot schedules via `ga_handler.py`

### Schedule Format

Robot schedules are lists of string targets: `"shelf3"`, `"goal0"`, `"home1"`. Pipe-delimited modifiers encode extra behavior:
- `"goal0|3"` — deliver exactly 3 items (for interrupted schedules)
- `"goal0|flag5"` — set flag after delivery (multi-robot synchronization)
- `"block|flag5"` — wait until flag is set before proceeding
- `"wait"` — idle (used in GA gene space)

### Genetic Algorithm (`scheduling/ga_encoding.py`, `scheduling/ga_handler.py`)

A genome encodes an **assignment**, not a list of stops. For an order of `n` items sorted by dependency (highest first):

```
genome[0:n]    cut bits    -- does item i open a new robot load?
genome[n:2n]   robot picks -- which robot carries load j?
```

Because a single order station only accepts decreasing dependency, each robot's load must be a *contiguous* block of the sorted item list and loads must be delivered in block order. This encoding makes that true by construction, and decoding repairs anything out of range — so **every genome is a schedule that can actually be built**, which is what gives the search a gradient. A robot may take several loads, so the search can choose between one robot making several trips and several robots making one each.

`decode_genome()` turns a genome into `[(robot_name, [Item, ...]), ...]` in delivery order. `Scheduler.emit_load_schedule()` (in `multi_robot.py`) turns that into the `block|flag` schedule, and is shared with `multi_robot_schedule_simple` so both modes emit the same structure.

`schedule_cost()` is the objective: makespan plus `ROBOT_TIME_WEIGHT` × robot-time. Makespan alone makes an extra robot look free and the search hands each order most of the fleet; the robot-time term is what accounts for the cost that falls on every *other* order. A robot blocked on another robot's flag is sent home (see `parse_schedule_value`), so waiting is charged the distance back from its home — waiting on your *own* previous delivery is free.

`GAHandler` is a singleton holding the per-search context, because pygad passes the fitness function nothing but the genome.

### Fault Tolerance

Four fault types with configurable rates (list of 4 floats: `[battery_critical, battery_low, actuator, sensor]`):
- **Battery critical**: Permanent failure (infinite wait)
- **Battery low**: Must return home, charge for 50 steps
- **Actuator**: Temporary 20-step halt
- **Sensor**: Robot ignores collision detection, 2-step halt

When `fault_tolerant_mode=True`, the scheduler calls `reassign_orders_if_faulted()` to generate replacement orders for remaining undelivered items.

### Deadlock Resolution

`Warehouse` handles three deadlock scenarios (implemented in `engine/deadlock.py`):
1. **Blocked path**: Robot recomputes A* path around obstacles after waiting
2. **Head-on collision**: Lower-priority robot moves aside (perpendicular)
3. **Cyclic deadlock**: Detected by following robot-target chains; broken by priority

### GUI (`mrws/io/gui.py`)

The PyQt6 GUI ("MRWS GUI") launches maximized and provides:
- **Central view**: zoomable/draggable warehouse grid (scroll to zoom, drag to pan, click to select robots)
- **Right dock panels**: Robot Inspector, Orders, Stats (top to bottom)
- **Stats panel**: line chart with toggleable series (Active, Backlog, Done, Idle, Faulted, Carrying) via checkboxes; x-axis = simulation steps; summary bar with utilization %, throughput, avg completion time, fault count, items in transit
- **Bottom toolbar**: Play/Pause (green/yellow highlight), Step, Reset, speed dropdown (0.1x–10.0x, default 1.0x), step counter, status
- **Keyboard shortcuts**: Space = play/pause toggle, S = step, R = reset

### Visualization Protocol

UDP JSON messages sent to Unity on `127.0.0.1:35891` via `mrws/io/udp.py`. Only transmits when `ROBOTSIM_TRANSMIT` env var is `"True"`.

### Warehouse File Format

Text files where each character represents a cell:
- `R`: Robot (and its home position)
- `S`: Shelf with item
- `G`: Order station (goal)
- `X`: Empty/traversable cell
- `W`: Wall

Warehouse files are in `simu/data/`.

## Critical Implementation Details

- **Item comparison**: Items use `__eq__` (name + dependency), NOT identity. `report_inventory()` and `transfer_inventory()` return `deepcopy` — never use `id()` for item comparison.
- **Robot position lookup**: `Warehouse._position_to_robot` maps `(x,y) -> robot_name` for O(1) lookups.
- **Schedule queues**: Robot `_movement_path` and schedules use `deque` — use `popleft()` not `pop(0)`.
- **GA scaling guard**: If >50 free robots, the genetic algorithm falls back to `multi_robot_schedule_simple` to avoid combinatorial explosion.
- **GA distance**: `GAHandler.get_distance_between()` computes taxicab distance on-demand from a positions dict.
- **GA objective weight**: `ROBOT_TIME_WEIGHT` in `ga_encoding.py` is empirical — swept against simulated step counts, then checked on held-out warehouses. Changing it changes how many robots the genetic scheduler commits per order.
- **Sensor fault zones**: `_faulty_blocked_cells` is a set of cells adjacent to sensor-faulted robots, recomputed each step.

## Dependencies

- Python >= 3.11
- pygad (genetic algorithm library)
- matplotlib (for result visualization)
- PyQt6 (for the GUI)
- Unity 6000.0.41f1 (for viz component, in `viz/` directory)
