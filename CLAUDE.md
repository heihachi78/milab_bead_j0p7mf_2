# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a PyBullet-based robotics simulation featuring a Franka Panda robot arm controlled by Claude AI (Anthropic) through natural language commands. The system supports both batch task execution and interactive conversational control.

## Running the Application

### Setup
```bash
# Install dependencies
pip install -r requirements.txt

# Configure environment variables (required)
# Create .env file with:
ANTHROPIC_API_KEY=your_key_here
ANTHROPIC_MODEL=claude-sonnet-4-5-20250929
```

### Three Execution Modes

1. **Batch Mode** (main.py) - Autonomous task execution with LLM validation
   ```bash
   python main.py --scene default
   python main.py --scene scene_01
   ```

2. **Interactive Console Mode** (main_console.py) - Terminal-based chat with continuous physics
   ```bash
   python main_console.py --scene default
   ```

3. **Interactive Streamlit Mode** (main_interactive.py) - Web-based chat interface
   ```bash
   streamlit run main_interactive.py
   ```

## Architecture

### Core Execution Flow (Batch Mode)

The batch mode (main.py) implements a sophisticated multi-stage LLM validation workflow:

1. **Scene Setup**: Load scene YAML, initialize PyBullet simulation, load objects
2. **Stabilization**: Stabilize robot physics and capture initial panorama
3. **Vision Analysis Stage**: LLM analyzes panorama image to identify object positions and reachability
4. **Spatial Analysis Stage**: LLM combines vision analysis with exact object positions to understand blocking relationships
5. **Planning Stage**: LLM generates initial execution plan using both vision and spatial analyses
6. **Validation Loop** (up to MAX_VALIDATION_ITERATIONS):
   - **Review Stage**: Critic LLM evaluates the plan against scene image and task
   - **Refinement Stage**: If rejected, planner LLM refines based on critique
   - Loop until plan is validated or max iterations reached
7. **Execution**: Execute validated plan using RobotController commands
8. **Cleanup**: Close logs and disconnect PyBullet

This multi-stage approach with separate vision/spatial analysis and iterative critique-refinement improves plan reliability.

### Interactive Mode Flow

Interactive modes (console and Streamlit) use a different architecture:

- **Tool-Based Control**: Claude responds to user messages using defined tools (pick_up, place, get_objects, etc.)
- **Conversational State**: Maintains conversation history for context-aware responses
- **Continuous Physics**: Console mode runs physics simulation during user input
- **Real-time Feedback**: Users can query state, request panoramas, and adjust commands dynamically

### Key Components

**src/llm_validator.py** - Multi-stage validation orchestrator
- `_analyze_scene_vision()`: Stage 1 - Vision-based analysis of panorama
- `_analyze_spatial_relationships()`: Stage 2 - Spatial reasoning with exact positions
- `_generate_initial_plan()`: Stage 3 - Generate plan from analyses
- `_critique_plan()`: Review stage - Validate plan against constraints
- `_refine_plan()`: Refinement stage - Fix plan based on critique
- `get_validated_plan()`: Main orchestration method
- Uses prompt caching for system prompts and panorama images to reduce costs

**src/llm_controller.py** - Plan execution engine
- `execute_plan()`: Executes validated command sequences (pick_up, place, place_on, rotate_gripper_90, reset_gripper_orientation)

**src/interactive_llm_controller.py** - Conversational control
- `handle_message()`: Process user messages with Claude's native tool calling
- Defines tools for robot actions, object queries, and panorama capture
- Implements prompt caching for system prompt and tools

**src/robot_controller.py** - Low-level robot control
- IK-based motion planning and execution
- Gripper control (open/close)
- High-level actions: `pick_up()`, `place()`, `place_on()`
- Orientation control: `rotate_orientation_90()`, `reset_orientation()`

**src/object_manager.py** - Object lifecycle management
- Load cubes with RGBA colors and positions
- Query object positions and dimensions
- Track object IDs

**src/camera_manager.py** - Multi-view panorama capture
- Captures 5 views (front, right, back, left, top)
- Stitches into single panorama image for LLM vision analysis

**src/scene_loader.py** - YAML scene parsing
- Loads scene configuration from scenes/*.yaml
- Defines metadata, objects, and task descriptions

**src/config.py** - Centralized configuration
- Robot parameters (joint limits, IK settings)
- Physics parameters (gravity, time steps)
- Movement thresholds and offsets
- Camera configuration
- LLM settings (MAX_VALIDATION_ITERATIONS, prompt caching)

**src/logger.py** - Comprehensive logging
- Dual logging: console (Rich) + file (rotating logs)
- LLM request/response tracking with token usage and cache metrics
- Simulation events and robot operations

### Prompt System

The system uses 11 different prompt templates (in prompts/):

**Batch Mode (5-stage workflow)**:
1. `vision_analysis_system_prompt.txt` / `vision_analysis_user_prompt.txt` - Analyze panorama for object visibility
2. `spatial_analysis_system_prompt.txt` / `spatial_analysis_user_prompt.txt` - Combine vision with positions
3. `planning_system_prompt.txt` / `planning_user_prompt.txt` - Generate execution plan
4. `review_system_prompt.txt` / `review_user_prompt.txt` - Critique plan validity
5. `refinement_system_prompt.txt` / `refinement_user_prompt.txt` - Refine based on critique

**Interactive Mode**:
- `interactive_system_prompt.txt` - Conversational robot control with tool definitions

### Scene Configuration

Scenes are defined in YAML files (scenes/*.yaml):
```yaml
metadata:
  name: "Scene Name"
  description: "Description"

objects:
  - name: "red_cube"
    type: "cube"
    position: [0.3, 0.3, 0.05]
    color: [1, 0, 0, 1]  # RGBA
    scale: 1.0

task:
  description: "Natural language task for batch mode"
```

Robot workspace: x: [-0.5, 0.7], y: [-0.7, 0.7], z: [0, 0.8] meters

## Key Configuration Parameters

**src/config.py** contains critical tuning parameters:

- `MAX_VALIDATION_ITERATIONS = 3` - Critique-refinement cycles in batch mode
- `STABILIZATION_LOOP_STEPS = 2500` - Physics stabilization before task execution
- `THRESHOLD_PRECISE = 0.001` - Position accuracy for robot movements (meters)
- `LINEAR_MOVEMENT_SPEED = 0.1` - Robot movement speed (m/s)
- `ENABLE_PROMPT_CACHING = True` - Use Anthropic prompt caching to reduce costs
- `PANORAMA_QUALITY = 75` - JPEG quality for panorama images (affects token usage)
- `INTERACTIVE_MAX_TOKENS = 4096` - Max tokens for interactive responses

## Development Notes

### Modifying Robot Actions

To add new robot commands:
1. Add method to `RobotController` (src/robot_controller.py)
2. For batch mode: Add action type to `LLMController.execute_plan()` and update planning prompts
3. For interactive mode: Add tool definition in `InteractiveLLMController._define_tools()` and tool handler

### Modifying Validation Logic

The validation workflow is in `LLMValidator.get_validated_plan()`:
- Adjust `MAX_VALIDATION_ITERATIONS` in config.py to change critique-refinement cycles
- Modify prompt templates in prompts/ to adjust LLM behavior
- Edit `_critique_plan()` or `_refine_plan()` to change validation logic

### Adding New Object Types

Currently only "cube" objects are supported. To add new types:
1. Update `ObjectManager.load_<type>()` to load URDF/geometry
2. Update scene loader to handle new type
3. Update planning prompts to describe new object capabilities

### Debugging

- Logs are written to `logs/` directory with timestamps
- Console output uses Rich formatting for clarity
- Set `LOG_API_CALLS = True` in config.py to log all LLM API calls
- Check `logs/api_calls_*.log` for detailed request/response traces
- Panoramas are saved to `images/` directory for inspection

### Prompt Caching

The system uses Anthropic's prompt caching to reduce costs:
- Vision analysis: Caches panorama image (reused across iterations)
- Review stage: Caches system prompt and panorama
- Interactive mode: Caches system prompt and tool definitions
- Cache TTL is 5 minutes (configurable via PROMPT_CACHE_TTL)
- Check logs for cache hit/miss metrics (cache_creation_tokens, cache_read_tokens)
