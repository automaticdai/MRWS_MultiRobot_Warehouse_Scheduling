# MRWS: MultiRobot Warehouse Scheduling Framework
MRWS is a Python-based simulator for a robotic smart warehouse, with many configurable properties. It supports connection to Unity for visualisation.

## Project Structure
```
├── README.md
├── data/: simulation-related data
├── requirements.txt
├── src/: Python source code
└── viz/: visualisation (Unity project)
```

## Requirements
Simulator:
- Python >= 3.11
- pygad
- matplotlib
- numpy

Visualisator:
- Unity == 6000..0.41f1

## Usage

Run `python main.py -t` to transmit positions to the visualiser.

Setting parameter `slow_for_transit` to `true` when calling `run_simulation()` on a simulator object may be useful to reduce the speed of the simulator for visualisation.
