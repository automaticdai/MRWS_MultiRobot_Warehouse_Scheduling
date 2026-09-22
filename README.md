# MRWS: A MultiRobot Warehouse Simulator

A Python simulator for a robotic smart warehouse, with many configurable properties. The aim is to develop and evaluate various path planning, scheduling and allocation algorithms.

## Project Structure

```text
Simulator (Python)
    |-- GUI (can be turned on/off)
    `-- Visualiser (Unity via TCP)
```

- `simu/`: Core simulation engine, scheduling logic, and CLI entrypoint.
- `viz/`: Unity project for real-time visualisation of simulation state.


## Requirements

Simulation:

- Python >= 3.11
- pygad
- matplotlib
- PyQt6 (for the debug GUI)

Visualisation:

- Unity 6000.3.5f1


## Usage

Run simulations from the `simu/` directory:

```bash
cd simu
python main.py                # Run 1000 simulations (default)
python main.py -n 1           # Run a single simulation
python main.py -t             # Run with transmission to Unity visualiser
python main.py -g             # Launch the MRWS GUI
python main.py -g -m multi-robot   # GUI with a specific scheduling mode
```

### CLI Parameters

| Flag | Long form | Description | Default |
|------|-----------|-------------|---------|
| `-n` | `--num-sims` | Number of simulation cycles to run | `1000` |
| `-m` | `--mode` | Scheduling mode (`simple`, `simple-interrupt`, `multi-robot`, `multi-robot-genetic`) | `simple-interrupt` |
| `-w` | `--warehouse` | Path to warehouse layout file | `data/whouse.txt` |
| `-t` | `--transmit` | Enable transmission to Unity visualiser (TCP) | off |
| `-g` | `--gui` | Launch PyQt6 debug GUI (ignores `-n`) | off |

### MRWS GUI

Launch with `-g` to open the MRWS GUI (PyQt6), which starts maximized:

```bash
python main.py -g
python main.py -g -m multi-robot   # with a specific scheduling mode
```

Features:
- **Grid view** — warehouse rendered with color-coded robots, shelves, goals, homes, and walls. Scroll to zoom, drag to pan. Click a robot to select it.
- **Robot inspector** (right dock) — selected robot's position, target, schedule queue, inventory, fault status, and A* path overlay.
- **Order panel** (right dock) — tracks active, backlog, and completed orders with robot assignments and item details.
- **Stats panel** (right dock, bottom) — line chart of simulation metrics over steps with toggleable series via checkboxes:
  - Active orders (green), Backlog (yellow), Completed (grey), Idle robots (blue), Faulted robots (red), Items carried (cyan)
  - Summary bar: utilization %, completed count, throughput, avg completion time, fault count, items in transit
- **Fault overlays** — robots change color by fault type (green=healthy, yellow=battery low, red=critical, orange=actuator, purple=sensor) with letter indicators.
- **Playback controls** — Play / Pause / Step / Reset buttons, speed dropdown (0.1x, 0.2x, 0.5x, **1.0x**, 2.0x, 5.0x, 10.0x), and step counter.
- **Keyboard shortcuts** — Space: play/pause, S: step, R: reset.

Use `-m` to select a scheduling mode: `simple`, `simple-interrupt` (default), `multi-robot`, or `multi-robot-genetic`.

When running with `-t` (without `-g`), setting `slow_for_transmit` to `True` when calling `run_simulation()` adds a delay between each simulation step so the Unity visualiser can keep up.


### Configurations

Simulations are configured by constructing a `Simulation` object in `main.py`:

```python
sim = Simulation(
    num_sims=1000,                       # Number of simulation runs
    whouse="whouse7x11.txt",             # Warehouse layout file
    num_items=10,                        # Number of distinct items (must match shelf count)
    inv_size=3,                          # Robot inventory capacity
    schedule_mode="simple-interrupt",    # Scheduling algorithm
    fault_rates=[0, 0, 0, 0],            # Fault rates: [battery_critical, battery_low, actuator, sensor]
    fault_mode=True,                     # Enable fault-tolerant rescheduling
    step_limit=1000                      # Max simulation steps before timeout
)
sim.run_simulation(reraise_error=True, slow_for_transmit=False)
```

### Scheduling Algorithms

- `simple` — one robot per order, FIFO assignment
- `simple-interrupt` — one robot per order with priority-based preemption
- `multi-robot` — multiple robots per order when beneficial
- `multi-robot-genetic` — uses a genetic algorithm (pygad) to optimise multi-robot schedules

### Warehouse Layout Files

Warehouse layouts are text files where each character represents a grid cell:

| Char | Meaning |
|------|---------|
| `R`  | Robot (and its home/charging position) |
| `S`  | Shelf containing an item |
| `G`  | Order station (goal/delivery point) |
| `X`  | Empty traversable cell |
| `W`  | Wall |

Example (`whouse7x11.txt`):

```
XXXXXXXXXXX
GXXSSXSSXXX
GRXSSXSSXXX
GRXXSXSXXXX
GRXXXXXXXXX
GRXXXXXXXXX
XXXXXXXXXXX
```

Custom layouts can be created as `.txt` files in the `simu/data/` directory. The number of `S` cells must equal the `num_items` parameter.

### Warehouse Tools

Two standalone scripts in `simu/` for generating and validating warehouse files:

**Generate a warehouse:**

```bash
python generate_warehouse.py --size 50 --robots 20 --items 50 --goals 5 -o ../data/whouse50x50.txt
```

| Flag | Default | Description |
|------|---------|-------------|
| `--size` | required | Side length (NxN) |
| `--robots` | required | Number of robots |
| `--items` | ~70% shelf capacity | Number of shelves/items |
| `--goals` | `robots` | Number of goal stations |
| `-o` | auto-named | Output file path |

**Validate a warehouse:**

```bash
python validate_warehouse.py ../data/whouse50x50.txt --items 50
```

**Scan all warehouses in `data/`:**

```bash
python validate_warehouse.py --scan-all            # structural info only
python validate_warehouse.py --scan-all --sim       # also run a simulation on each
python validate_warehouse.py --scan-all --sim --mode simple   # pick scheduling mode
```

Example output:

```
┌───────────────────┬─────────┬────────┬─────────┬───────┬───────┬───────────┐
│ File              │ Size    │ Robots │ Shelves │ Goals │ Valid │ Sim Steps │
├───────────────────┼─────────┼────────┼─────────┼───────┼───────┼───────────┤
│ whouse.txt        │ 11x7    │ 4      │ 10      │ 5     │ YES   │ 156       │
├───────────────────┼─────────┼────────┼─────────┼───────┼───────┼───────────┤
│ whouse100x100.txt │ 100x100 │ 40     │ 100     │ 10    │ YES   │ 530       │
├───────────────────┼─────────┼────────┼─────────┼───────┼───────┼───────────┤
│ whouse10x10.txt   │ 10x10   │ 3      │ 10      │ 2     │ YES   │ 231       │
├───────────────────┼─────────┼────────┼─────────┼───────┼───────┼───────────┤
│ whouse20x20.txt   │ 20x20   │ 5      │ 15      │ 3     │ YES   │ 347       │
├───────────────────┼─────────┼────────┼─────────┼───────┼───────┼───────────┤
│ whouse50x50.txt   │ 50x50   │ 20     │ 50      │ 5     │ YES   │ 504       │
└───────────────────┴─────────┴────────┴─────────┴───────┴───────┴───────────┘
```

### Fault Simulation

Robot fault rates are specified as a list of four probabilities (per step, per robot):

1. **Battery critical** — permanent failure, robot becomes inoperable
2. **Battery low** — robot must return home and charge before resuming
3. **Actuator** — temporary movement halt (overheating)
4. **Sensor** — robot can no longer detect surroundings

Set all to `0` for a fault-free simulation. When `fault_mode=True`, the scheduler reassigns orders from faulted robots.

### Batch Experiments

`main.py` includes helper functions for running batch experiments with matplotlib visualisation:

- `run_completion_time_test(fault_rates, num_sims=500, whouse=...)` — compares all four scheduling algorithms
- `run_fault_test(scheduling_mode, num_sims=250, whouse=...)` — compares fault-tolerant vs non-fault-tolerant strategies
- `run_simulation_performance_test(scheduling_mode, robots_max, size_max, step_limit)` — benchmarks step time across varying warehouse sizes and robot counts
