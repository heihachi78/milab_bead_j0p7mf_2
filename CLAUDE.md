# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a PyBullet-based Franka Panda robotic arm simulator with LLM-powered control using Anthropic's Claude API. The system supports two modes:

1. **Batch Mode** ([main.py](main.py)): Executes predefined tasks from YAML scene configurations with automatic plan generation and validation
2. **Interactive Mode** ([main_interactive.py](main_interactive.py)): Streamlit-based chat interface for conversational robot control using Claude's tool calling API

## Setup and Running

### Prerequisites
- Python 3.8+
- PyBullet 3.2.5+
- Anthropic API key

### Installation
```bash
pip install -r requirements.txt
```

### Environment Configuration
Create a `.env` file in the project root:
```
ANTHROPIC_API_KEY=your_api_key_here
ANTHROPIC_MODEL=claude-sonnet-4-5-20250929
```

### Running the Simulation

**Batch Mode** (executes scene task automatically):
```bash
python main.py --scene default
python main.py --scene scene_1
```

**Interactive Mode** (Streamlit chat interface):
```bash
streamlit run main_interactive.py
```

Available scenes can be found in [scenes/](scenes/) directory.

## Core Architecture

### Key Components

The system follows a modular architecture with clear separation of concerns:

- **RobotController** ([src/robot_controller.py](src/robot_controller.py)): Low-level robot control, IK solving, gripper operations, and pick-and-place primitives. Handles PyBullet initialization and connection modes (GUI, headless, or shared memory).

- **ObjectManager** ([src/object_manager.py](src/object_manager.py)): Object registry, URDF loading, position/dimension queries. Maintains a dictionary mapping object names to PyBullet IDs.

- **CameraManager** ([src/camera_manager.py](src/camera_manager.py)): Multi-angle panorama capture (front, right, back, left, top). Images saved to [images/](images/) folder with timestamps.

- **SimulationState** ([src/simulation_state.py](src/simulation_state.py)): Tracks simulation time and state.

- **LLMController** ([src/llm_controller.py](src/llm_controller.py)): Executes validated command plans in batch mode. Commands: `pick_up`, `place`, `place_on`, `rotate_orientation_90`, `reset_orientation`.

- **LLMValidator** ([src/llm_validator.py](src/llm_validator.py)): Plan generation and validation loop for batch mode. Uses a critique-refinement cycle with separate planning and review prompts. Maximum iterations configured via `MAX_VALIDATION_ITERATIONS` in [src/config.py](src/config.py).

- **InteractiveLLMController** ([src/interactive_llm_controller.py](src/interactive_llm_controller.py)): Claude native tool calling for conversational robot control. Defines tools for object queries, gripper control, movement commands, and panorama capture. Manages conversation history and token usage.

### Configuration

All configuration is centralized in [src/config.py](src/config.py):
- Robot kinematics parameters (joint limits, IK solver settings)
- Physics parameters (gravity, motor forces, simulation timing)
- Movement thresholds and offsets for pick/place operations
- Camera settings (resolution, FOV, panorama angles)
- LLM settings (validation iterations, max tokens, prompt caching)
- Logging configuration

### Scene Configuration

Scenes are defined in YAML files in [scenes/](scenes/) directory. Each scene specifies:
- **metadata**: Name and description
- **objects**: List of objects with name, type (currently only "cube"), position, color (RGBA), and scale
- **task**: Natural language description for batch mode

See [scenes/README.md](scenes/README.md) for detailed format and creation guidelines.

### LLM Prompts

Prompt templates are stored in [prompts/](prompts/) directory:
- **Batch mode**: `planning_system_prompt.txt`, `planning_user_prompt.txt`, `review_system_prompt.txt`, `review_user_prompt.txt`, `refinement_system_prompt.txt`, `refinement_user_prompt.txt`
- **Interactive mode**: `interactive_system_prompt.txt`

Prompts use placeholders that are filled at runtime with object positions, dimensions, and scene information.

## Control Flow

### Batch Mode Workflow
1. Load scene configuration from YAML
2. Initialize PyBullet (GUI mode with direct window opening to avoid macOS shared memory hang)
3. Load objects into simulation
4. Stabilize robot and capture panorama
5. **LLMValidator** generates initial plan using `planning_system_prompt.txt` with panorama as vision input
6. **LLMValidator** critiques plan using `review_system_prompt.txt`
7. If critique fails, refine plan using `refinement_system_prompt.txt` (up to `MAX_VALIDATION_ITERATIONS`)
8. If validation succeeds, **LLMController** executes the validated plan sequentially
9. Exit with status code 0 (success) or 1 (validation failure)

### Interactive Mode Workflow
1. Initialize PyBullet connection (direct GUI mode)
2. Load simulation components and discover objects
3. Launch Streamlit web interface
4. User sends message via chat
5. **InteractiveLLMController** calls Claude API with tool definitions
6. Claude may use tools (get object positions, move gripper, capture panorama, etc.)
7. Tool results fed back to Claude for final response
8. Conversation history maintained in session state
9. Token usage tracked and displayed in sidebar

### Robot Movement Primitives

From [src/robot_controller.py](src/robot_controller.py):
- `move_to_target(position, threshold)`: Move end effector to target position using IK
- `move_to_target_smooth(position, threshold)`: Smooth linear interpolated movement
- `pick_up(object_name)`: Complete pick-up sequence (approach, grasp, lift)
- `place(position)`: Place held object at absolute position
- `place_on(object_name)`: Place held object on top of target object
- `rotate_orientation_90()`: Rotate gripper 90 degrees (for stacking)
- `reset_orientation()`: Reset gripper to default downward orientation
- `open_gripper()` / `close_gripper()`: Gripper control

## Platform-Specific Notes

### macOS Compatibility
- PyBullet shared memory connection can hang on macOS
- Batch mode uses direct GUI connection (`mode='gui'`) instead of shared memory attempt
- Debug visualizer configuration skipped in headless mode to avoid hangs
- See [src/robot_controller.py](src/robot_controller.py):16-66 for initialization logic

## Development Guidelines

### Adding New Object Types
1. Extend [src/object_manager.py](src/object_manager.py) `load_<type>()` method
2. Update [src/scene_loader.py](src/scene_loader.py) to handle new type in validation
3. Modify scene loading loop in [main.py](main.py):68-75 to load new type

### Modifying LLM Behavior
- **Batch mode**: Edit prompt files in [prompts/](prompts/) directory
- **Interactive mode**: Edit `interactive_system_prompt.txt` or modify tool definitions in [src/interactive_llm_controller.py](src/interactive_llm_controller.py):81-87

### Adjusting Movement Precision
Threshold constants in [src/config.py](src/config.py):
- `THRESHOLD_OVER_TARGET`: Reaching waypoints above objects (0.025m)
- `THRESHOLD_CLOSE_TARGET`: Approaching objects (0.01m)
- `THRESHOLD_PRECISE`: Final placement precision (0.001m)
- `THRESHOLD_PRECISE_STRICT`: Strictest precision (0.0005m)

### Logging
- Application logs: [logs/](logs/) directory with timestamped session files
- API call logs (interactive mode): `logs/api_calls_{session_name}.log` (if `LOG_API_CALLS=True`)
- Console output controlled via `SimulationLogger` methods
