# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an LLM-controlled PyBullet robotics simulation for a Franka Panda robotic arm. The system uses Anthropic's Claude API to generate, validate, and execute pick-and-place task plans based on natural language descriptions and visual scene understanding through multi-camera panoramas.

## Core Architecture

### Modular Component System

The codebase follows a clean separation of concerns with distinct manager classes:

- **[main.py](main.py)**: Orchestration script that initializes all components and runs the LLM-driven execution pipeline
- **[src/config.py](src/config.py)**: Centralized configuration with all parameters (physics, robot, camera, LLM settings)
- **[src/robot_controller.py](src/robot_controller.py)**: Robot control logic (IK, motion planning, gripper operations, pick/place primitives)
- **[src/object_manager.py](src/object_manager.py)**: Object loading, tracking, and position/dimension queries
- **[src/camera_manager.py](src/camera_manager.py)**: Multi-camera panorama capture from 5 viewpoints (front/right/back/left/top)
- **[src/simulation_state.py](src/simulation_state.py)**: Simulation state tracking (time, debug visualization)
- **[src/llm_controller.py](src/llm_controller.py)**: Executes validated action plans using robot primitives
- **[src/llm_validator.py](src/llm_validator.py)**: LLM-based plan generation and iterative validation workflow
- **[src/logger.py](src/logger.py)**: Comprehensive logging for app events, LLM interactions, and robot operations

### LLM Validation Workflow

The system uses a critique-refinement loop for plan validation ([src/llm_validator.py](src/llm_validator.py:357-412)):

1. **Planning**: Generate initial plan from task description and panorama image
2. **Review**: Critique plan for feasibility and correctness
3. **Refinement**: Refine plan based on critique feedback
4. Repeat review-refinement up to `MAX_VALIDATION_ITERATIONS` (default: 3)
5. **Execution**: Execute validated plan via `LLMController`

Each phase uses dedicated prompt templates from [prompts/](prompts/):
- `planning_system_prompt.txt` / `planning_user_prompt.txt`
- `review_system_prompt.txt` / `review_user_prompt.txt`
- `refinement_system_prompt.txt` / `refinement_user_prompt.txt`

### Action Plan Format

Plans are JSON structures with a `commands` array. Supported actions:
- `pick_up`: `{"action": "pick_up", "object": "cube_name"}`
- `place`: `{"action": "place", "position": [x, y, z]}`
- `place_on`: `{"action": "place_on", "object": "target_cube"}`

See [src/llm_controller.py](src/llm_controller.py:67-113) for execution logic.

### Robot Control Hierarchy

**Movement primitives** ([src/robot_controller.py](src/robot_controller.py)):
- `move_to_target()`: Low-level IK-based movement until convergence threshold is met
- `move_to_target_smooth()`: Smooth linear interpolation movement at constant velocity (uses `LINEAR_MOVEMENT_SPEED` from config)

**High-level operations**:
- `pick_up(object_name)`: Multi-phase pick sequence (approach → open gripper → grasp → lift) with automatic panorama capture
- `place(position)`: Place at absolute position [x, y, z]
- `place_on(object_name)`: Place on top of another object

Each operation uses different threshold values for different phases (over target, close target, precise) defined in [src/config.py](src/config.py:93-97).

### Camera System

[src/camera_manager.py](src/camera_manager.py) captures 5-viewpoint panoramas:
- 4 horizontal views (0°, 90°, 180°, 270°)
- 1 top-down view (-89° pitch)

Panoramas are saved after each robot operation with sequential numbering: `panorama_NNN_operation_name.png`

The LLM receives these panoramas for visual scene understanding during plan generation and validation.

## Development Commands

### Running the Simulation

```bash
python main.py
```

Launches PyBullet GUI and executes the LLM-driven task pipeline:
1. Initialize simulation and stabilize robot
2. Load colored cubes into scene
3. Capture initial panorama
4. Load task from [prompts/llm_task_init.txt](prompts/llm_task_init.txt)
5. Generate and validate plan using Claude API
6. Execute validated plan
7. Save logs and close

### Installing Dependencies

```bash
pip install -r requirements.txt
```

Required dependencies include: `pybullet`, `anthropic`, `numpy`, `pillow`, `python-dotenv`

### Virtual Environment

```bash
source .venv/bin/activate
```

### Environment Configuration

Create a `.env` file with:
```
ANTHROPIC_API_KEY=your_api_key_here
ANTHROPIC_MODEL=claude-3-5-haiku-20241022
```

The `ANTHROPIC_MODEL` can be overridden per-validation in [src/config.py](src/config.py:133) via `VALIDATION_MODEL`.

## Key Configuration Parameters

### LLM Settings ([src/config.py](src/config.py:131-133))

- `MAX_VALIDATION_ITERATIONS`: Maximum critique-refinement cycles (default: 3)
- `VALIDATION_MODEL`: Override model for validation (None = use `.env` model)

### Robot Control Tuning

- **Thresholds** (`THRESHOLD_OVER_TARGET`, `THRESHOLD_CLOSE_TARGET`, `THRESHOLD_PRECISE`, `THRESHOLD_PRECISE_STRICT`): Multi-phase precision control
- **IK solver** (`IK_MAX_ITERATIONS`, `IK_RESIDUAL_THRESHOLD`): Balance solution quality vs. speed
- **Motor forces** (`ARM_MOTOR_FORCE`, `GRIPPER_MOTOR_FORCE`): Movement speed and manipulation capability
- **Movement speed** (`LINEAR_MOVEMENT_SPEED`): Velocity for smooth linear movements (default: 0.01 m/s)

### Pick-and-Place Offsets

Vertical approach strategy defined by offset constants:
- `OVER_TARGET_Z`: High overhead position (0.5m)
- `PICK_CLOSE_OFFSET`: Intermediate approach height
- `PLACE_ON_TARGET_OFFSET`: Final placement height for stacking

### Camera Configuration ([src/config.py](src/config.py:115-129))

- `CAMERA_YAW_ANGLES` / `CAMERA_PITCH_ANGLES`: 5-viewpoint angles
- `CAMERA_IMAGE_WIDTH` / `CAMERA_IMAGE_HEIGHT`: Resolution (default: 640x480)
- `CAMERA_DISTANCE` / `CAMERA_FOV`: View parameters

### Logging ([src/config.py](src/config.py:135-137))

- `LOGS_FOLDER`: Log directory (default: 'logs')
- `LOG_SESSION_NAME`: Custom session name (None = auto-timestamp)

## Common Modifications

### Changing the Task

Edit [prompts/llm_task_init.txt](prompts/llm_task_init.txt) to specify a new task in natural language.

Example: "Create a stack that is in this order from bottom to up: blue cube, yellow cube, green cube, purple cube, red cube"

### Adding New Objects

1. Load object in [main.py](main.py:68-73) using `object_manager.load_cube()`
2. Update prompt templates to include new object names in `OBJECTS_LIST`
3. Object positions and dimensions are automatically queried

### Modifying LLM Prompts

Edit files in [prompts/](prompts/):
- **Planning prompts**: Control how initial plans are generated
- **Review prompts**: Control how plans are critiqued
- **Refinement prompts**: Control how plans are improved

Prompts use template variables like `{OBJECTS_LIST}`, `{OBJECTS_INFO}`, `{TASK_DESCRIPTION}`.

### Adjusting Validation Rigor

Increase `MAX_VALIDATION_ITERATIONS` in [src/config.py](src/config.py:132) for more thorough validation at the cost of additional API calls.

### Using Different Claude Models

Change `ANTHROPIC_MODEL` in `.env` or set `VALIDATION_MODEL` in [src/config.py](src/config.py:133). Available models include Haiku, Sonnet, and Opus variants.

## Debugging

### Visualization

Debug trails are automatically drawn when `hasPrevPose=1` in simulation state:
- Blue line (`DEBUG_LINE_COLOR_1`): Target trajectory
- Red line (`DEBUG_LINE_COLOR_2`): Actual end-effector trajectory

Control persistence with `TRAIL_DURATION` in [src/config.py](src/config.py:51).

### Logging

Logs are saved to `logs/` directory with timestamped filenames. The [src/logger.py](src/logger.py) provides:
- `app_logger`: Application-level events (file handler)
- Console output methods: `console_info()`, `console_progress()`, etc.
- Structured logging for LLM requests/responses, robot operations, and validation workflow

### Panorama Images

Captured panoramas are saved to `images/` with sequential numbering. Review these to understand what the LLM sees during plan generation.

## Architecture Notes

**Configuration-first design**: All parameters live in [src/config.py](src/config.py). Never hardcode values in implementation files.

**State management**: [src/simulation_state.py](src/simulation_state.py) maintains simulation time and debug visualization state, passed through all movement operations.

**Object registry**: [src/object_manager.py](src/object_manager.py) maintains a `name → PyBullet ID` mapping. Always use object names, not IDs, in high-level code.

**Prompt-driven behavior**: The system's task execution is entirely driven by LLM interpretation of prompts. Changing prompts changes behavior without code modifications.
