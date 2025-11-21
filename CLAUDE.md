# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a PyBullet-based robotics simulation project for a Franka Panda robotic arm. The simulation demonstrates pick-and-place operations with multiple colored cubes using inverse kinematics (IK).

## Core Architecture

### Main Components

- **[main.py](main.py)**: Primary simulation script containing the complete robot control logic
- **[config.py](config.py)**: Centralized configuration file with all simulation parameters, physical constants, and tuning values

### Key Design Patterns

**Configuration-Based Architecture**: All parameters are externalized to [config.py](config.py). When modifying robot behavior, joint limits, physics constants, or movement thresholds, always update [config.py](config.py) rather than hardcoding values in [main.py](main.py).

**State Propagation**: The simulation maintains state through function return tuples: `(t, hasPrevPose, prevPose, prevPose1, gripper_pos)`. All movement functions must accept and return these values to maintain debug visualization and timing consistency.

**Hierarchical Movement API**:
- `move_to_target()`: Low-level function that moves end effector to a position until convergence threshold is met
- `move_to_target_linear()`: Higher-level function that creates intermediate waypoints for straighter trajectories
- `pick_up()`: Complete pick operation orchestrating multiple movements and gripper actions
- `place()`: Complete place operation orchestrating multiple movements and gripper actions

### Object Management

The `objects` dictionary (lines 100-104 in [main.py](main.py)) maps object names to PyBullet object IDs. Object dimensions are queried dynamically using `get_object_dimensions()` rather than being hardcoded, ensuring accuracy even when scaling changes.

## Development Commands

### Running the Simulation

```bash
python main.py
```

This launches the PyBullet GUI and runs the pick-and-place demonstration.

### Installing Dependencies

```bash
pip install -r requirements.txt
```

### Activating Virtual Environment

```bash
source .venv/bin/activate
```

## Key Configuration Parameters

### Robot Control Tuning

- **Movement thresholds** (`THRESHOLD_OVER_TARGET`, `THRESHOLD_CLOSE_TARGET`, `THRESHOLD_PRECISE`): Control when the robot considers a target reached. Smaller values increase precision but may cause longer convergence times.
- **IK solver settings** (`IK_MAX_ITERATIONS`, `IK_RESIDUAL_THRESHOLD`): Balance between solution quality and computation time.
- **Motor forces** (`ARM_MOTOR_FORCE`, `GRIPPER_MOTOR_FORCE`): Affect movement speed and ability to manipulate objects.

### Pick-and-Place Offsets

The `PICK_Z_OFFSET_*` and `PLACE_Z_OFFSET_*` parameters define the vertical approach strategy. The robot moves to an overhead position, descends to a close position, then makes a precise final approach with linear interpolation.

### Simulation Behavior

- `USE_SIMULATION`: Set to 0 to directly apply IK solutions without physics (useful for testing IK accuracy)
- `USE_REAL_TIME_SIMULATION`: Enable for real-time mode
- `TRAIL_DURATION`: Controls how long debug visualization lines persist (0 for permanent)

## Common Modifications

### Adding New Objects

1. Load the URDF in [main.py](main.py) (see lines 67-74)
2. Add to the `objects` dictionary with a descriptive name
3. Use `get_object_center_position('object_name')` to reference it in pick/place operations

### Changing Robot Behavior

Modify the main loop (lines 433-446) which currently:
1. Picks blue cube → places on red cube
2. Picks green cube → places on blue cube

### Adjusting Movement Quality

- Increase `DEFAULT_NUM_WAYPOINTS` for smoother linear trajectories
- Decrease `DISTANCE_CHANGE_THRESHOLD` for more precise positioning
- Adjust threshold parameters for different accuracy requirements per operation phase

## Debugging

Debug visualization is automatically drawn when `hasPrevPose=1`. Two colored lines show:
- Blue line (`DEBUG_LINE_COLOR_1`): End effector trajectory
- Red line (`DEBUG_LINE_COLOR_2`): Link state trajectory

Control persistence with `TRAIL_DURATION` in [config.py](config.py).
