# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an LLM-controlled PyBullet robotics simulation for a Franka Panda robotic arm. The system uses Anthropic's Claude API to generate, validate, and execute pick-and-place task plans based on natural language descriptions and visual scene understanding through multi-camera panoramas.

**The project supports two modes:**
1. **Batch Mode** ([main.py](main.py)): Automated task execution with plan validation
2. **Interactive Mode** ([main_interactive.py](main_interactive.py)): Real-time chat interface for conversational robot control

## Core Architecture

### Modular Component System

The codebase follows a clean separation of concerns with distinct manager classes:

- **[main.py](main.py)**: Orchestration script that initializes all components and runs the LLM-driven execution pipeline
- **[src/config.py](src/config.py)**: Centralized configuration with all parameters (physics, robot, camera, LLM settings)
- **[src/scene_loader.py](src/scene_loader.py)**: YAML-based scene configuration loader (objects and tasks)
- **[src/robot_controller.py](src/robot_controller.py)**: Robot control logic (IK, motion planning, gripper operations, pick/place primitives)
- **[src/object_manager.py](src/object_manager.py)**: Object loading, tracking, and position/dimension queries
- **[src/camera_manager.py](src/camera_manager.py)**: Multi-camera panorama capture from 5 viewpoints (front/right/back/left/top)
- **[src/simulation_state.py](src/simulation_state.py)**: Simulation state tracking (time, debug visualization)
- **[src/llm_controller.py](src/llm_controller.py)**: Executes validated action plans using robot primitives (batch mode)
- **[src/llm_validator.py](src/llm_validator.py)**: LLM-based plan generation and iterative validation workflow (batch mode)
- **[src/interactive_llm_controller.py](src/interactive_llm_controller.py)**: Interactive conversational controller with native tool calling (interactive mode)
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

### Scene Configuration System

[src/scene_loader.py](src/scene_loader.py) manages YAML-based scene definitions from the [scenes/](scenes/) directory:
- **Scene metadata**: Name and description
- **Object definitions**: Position, color, type, and scale for each object
- **Task description**: Natural language task for batch mode execution

Scene files use a validated YAML schema with automatic error checking. Both batch and interactive modes support the `--scene` CLI argument to load different configurations.

### Camera System

[src/camera_manager.py](src/camera_manager.py) captures 5-viewpoint panoramas:
- 4 horizontal views (0°, 90°, 180°, 270°)
- 1 top-down view (-89° pitch)

Panoramas are saved after each robot operation with sequential numbering: `panorama_NNN_operation_name.png`

The LLM receives these panoramas for visual scene understanding during plan generation and validation.

## Development Commands

### Running Batch Mode

```bash
python main.py
python main.py --scene default
python main.py --scene example_stacking
```

Launches PyBullet GUI and executes the LLM-driven task pipeline:
1. Initialize simulation and stabilize robot
2. Load scene configuration from YAML file (objects and task)
3. Load objects into scene
4. Capture initial panorama
5. Generate and validate plan using Claude API
6. Execute validated plan
7. Save logs and close

**CLI Arguments:**
- `--scene SCENE_NAME`: Specify which scene to load (default: `default`)

### Running Interactive Mode

```bash
streamlit run main_interactive.py
streamlit run main_interactive.py -- --scene default
streamlit run main_interactive.py -- --scene example_stacking
```

Launches a Streamlit web interface with:
1. PyBullet GUI for visual feedback
2. Web chat interface at http://localhost:8501
3. Real-time conversational robot control
4. Natural language command execution via Claude LLM
5. Sidebar with live scene information (object positions, gripper state)

**Interactive Mode Features:**
- Chat with Claude to control the robot in natural language
- LLM uses 11 tools to query scene state and execute operations
- On-demand panorama capture (only when LLM requests visual info)
- Tool execution results displayed inline in chat
- Conversation history maintained throughout session

### Installing Dependencies

```bash
pip install -r requirements.txt
```

Required dependencies include: `pybullet`, `anthropic`, `numpy`, `pillow`, `python-dotenv`, `streamlit`

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

### Creating Custom Scenes

Scenes are defined in YAML files in the [scenes/](scenes/) directory. Each scene contains object definitions and a task description.

**Create a new scene file** (e.g., `scenes/my_scene.yaml`):

```yaml
metadata:
  name: "My Custom Scene"
  description: "Description of your scene"

objects:
  - name: "red_cube"
    type: "cube"
    position: [0.3, 0.4, 0.05]  # [x, y, z] in meters
    color: [1, 0, 0, 1]  # [r, g, b, a] (0.0-1.0)
    scale: 1.0

  - name: "blue_cube"
    type: "cube"
    position: [0.3, 0.2, 0.05]
    color: [0, 0, 1, 1]
    scale: 1.0

task:
  description: "Stack the red cube on top of the blue cube"
```

**Run with your custom scene:**
```bash
python main.py --scene my_scene
streamlit run main_interactive.py -- --scene my_scene
```

See [scenes/README.md](scenes/README.md) for detailed format documentation.

### Changing the Task

Edit the `task.description` field in your scene YAML file.

Example: `"Create a stack that is in this order from bottom to up: blue cube, yellow cube, green cube, purple cube, red cube"`

### Adding New Objects

Add new object entries to the `objects` list in your scene YAML file. Object positions and dimensions are automatically queried by the LLM system.

### Modifying LLM Prompts

Edit files in [prompts/](prompts/):
- **Planning prompts**: Control how initial plans are generated
- **Review prompts**: Control how plans are critiqued
- **Refinement prompts**: Control how plans are improved

Prompts use template variables like `{OBJECTS_LIST}`, `{OBJECTS_INFO}`, `{TASK_DESCRIPTION}`.

### Using Interactive Mode

Launch the Streamlit interface:
```bash
streamlit run main_interactive.py
```

The web interface opens at `http://localhost:8501` with a chat interface. Example commands:
- "What objects are in the scene?"
- "Pick up the blue cube"
- "Show me the scene" (triggers panorama capture)
- "Move the gripper to [0.3, 0.4, 0.2]"
- "Stack the red cube on the blue cube"

The LLM has access to 12 tools:
1. **Query Tools**: `get_gripper_position`, `get_gripper_state`, `get_object_position`, `get_all_objects`, `get_panorama`
2. **Movement Tools**: `move_gripper`, `move_gripper_smooth`
3. **Gripper Tools**: `open_gripper`, `close_gripper`
4. **High-Level Tools**: `pick_up_object`, `place_object`, `place_on_object`

Tool results are displayed inline in the chat interface. The sidebar shows real-time scene information.

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
- `llm_logger`: LLM requests and responses (batch mode)
- `robot_logger`: Robot operations and movements
- `interactive_logger`: Interactive chat messages and tool calls (interactive mode)
- Console output methods: `console_info()`, `console_progress()`, etc.

Structured logging includes LLM requests/responses, robot operations, validation workflow, and interactive tool usage.

### Panorama Images

Captured panoramas are saved to `images/` with sequential numbering. Review these to understand what the LLM sees during plan generation.

## Architecture Notes

**Configuration-first design**: All parameters live in [src/config.py](src/config.py). Never hardcode values in implementation files.

**Scene-based configuration**: Objects and tasks are defined in YAML files ([scenes/](scenes/)), separating data from code. The [src/scene_loader.py](src/scene_loader.py) module handles loading and validation.

**State management**: [src/simulation_state.py](src/simulation_state.py) maintains simulation time and debug visualization state, passed through all movement operations.

**Object registry**: [src/object_manager.py](src/object_manager.py) maintains a `name → PyBullet ID` mapping. Always use object names, not IDs, in high-level code.

**Prompt-driven behavior**: The system's task execution is entirely driven by LLM interpretation of prompts. Changing prompts changes behavior without code modifications.

**Dual execution modes**:
- **Batch mode** ([main.py](main.py)): Single-task execution with validation pipeline using JSON action plans. Task loaded from scene YAML file.
- **Interactive mode** ([main_interactive.py](main_interactive.py)): Conversational control using Claude's native tool calling API with 11 available tools. System prompt defined in [prompts/interactive_system_prompt.txt](prompts/interactive_system_prompt.txt).
