# PyBullet Franka Panda LLM-Controlled Robot Simulator

A PyBullet-based robotic arm simulator with LLM-powered control using Anthropic's Claude API. The system provides three distinct operational modes for automated task execution and interactive robot control through natural language.

## Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [Configuration](#configuration)
- [Operational Modes](#operational-modes)
  - [Batch Mode (main.py)](#1-batch-mode-mainpy)
  - [Console Interactive Mode (main_console.py)](#2-console-interactive-mode-main_consolepy)
  - [Streamlit Interactive Mode (main_interactive.py)](#3-streamlit-interactive-mode-main_interactivepy)
- [LLM Integration Details](#llm-integration-details)
  - [Batch Mode LLM Workflow](#batch-mode-llm-workflow)
  - [Interactive Mode LLM Workflow](#interactive-mode-llm-workflow)
  - [Token Usage and Caching](#token-usage-and-caching)
- [Scenes](#scenes)
- [Prompts](#prompts)
- [Logging](#logging)
- [Robot Commands](#robot-commands)
- [Project Structure](#project-structure)
- [Development](#development)

## Features

- **Three Operational Modes**: Batch execution, console-based chat, and web-based Streamlit interface
- **LLM-Powered Control**: Natural language task planning and execution using Claude API
- **Automated Plan Validation**: Self-critique and refinement loop for robust task plans
- **Interactive Tool Calling**: Claude's native tool calling for conversational robot control
- **Multi-Camera Vision**: 5-viewpoint panorama capture for scene understanding
- **Precise Manipulation**: IK-based movement with multiple precision thresholds
- **Comprehensive Logging**: Timestamped logs with structured LLM call tracking
- **Prompt Caching**: Efficient token usage through Anthropic's prompt caching

## System Architecture

### Core Components

- **RobotController** ([src/robot_controller.py](src/robot_controller.py)): Low-level robot control, IK solving, gripper operations, pick-and-place primitives
- **ObjectManager** ([src/object_manager.py](src/object_manager.py)): Object registry, URDF loading, position/dimension queries
- **CameraManager** ([src/camera_manager.py](src/camera_manager.py)): Multi-angle panorama capture (front, right, back, left, top)
- **SimulationState** ([src/simulation_state.py](src/simulation_state.py)): Tracks simulation time and state
- **LLMController** ([src/llm_controller.py](src/llm_controller.py)): Executes validated command plans (batch mode)
- **LLMValidator** ([src/llm_validator.py](src/llm_validator.py)): Plan generation and validation loop (batch mode)
- **InteractiveLLMController** ([src/interactive_llm_controller.py](src/interactive_llm_controller.py)): Claude native tool calling for conversational control (interactive modes)

### Configuration

All configuration is centralized in [src/config.py](src/config.py):
- Robot kinematics parameters (joint limits, IK solver settings)
- Physics parameters (gravity, motor forces, simulation timing)
- Movement thresholds and offsets for pick/place operations
- Camera settings (resolution, FOV, panorama angles)
- LLM settings (validation iterations, max tokens, prompt caching)
- Logging configuration

## Installation

### Prerequisites

- Python 3.8 or higher
- PyBullet 3.2.5+
- Anthropic API key

### Setup Steps

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd milab_bead_j0p7mf_2
   ```

2. **Create and activate a virtual environment** (recommended):
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**:
   Create a `.env` file in the project root:
   ```env
   ANTHROPIC_API_KEY=your_api_key_here
   ANTHROPIC_MODEL=claude-sonnet-4-5-20250929
   ```

### Requirements Breakdown

**Core Dependencies**:
- `pybullet>=3.2.5` - Physics simulation engine
- `anthropic>=0.25.0` - Claude API client
- `numpy>=1.24.0` - Numerical computations
- `pillow>=10.0.0` - Image processing
- `pyyaml>=6.0` - YAML scene configuration parsing
- `python-dotenv>=1.0.0` - Environment variable management

**Interactive Mode Dependencies**:
- `streamlit>=1.28.0` - Web-based chat interface
- `rich>=13.0.0` - Terminal-based chat interface with formatting

**Optional Dependencies**:
- `scipy>=1.11.0` - Scientific computing utilities
- `opencv-python>=4.8.0` - Advanced image processing
- `jsonschema>=4.19.0` - JSON validation
- `pytest>=7.4.0` - Testing framework

## Configuration

### Environment Variables

Create a `.env` file:
```env
ANTHROPIC_API_KEY=your_api_key_here
ANTHROPIC_MODEL=claude-sonnet-4-5-20250929
```

### Key Configuration Parameters

Edit [src/config.py](src/config.py) to customize:

**Robot Configuration**:
- `END_EFFECTOR_INDEX`: End effector link index (default: 11)
- `JOINT_LIMITS`: Joint angle limits for IK
- `IK_MAX_ITERATIONS`: Maximum IK solver iterations

**Movement Thresholds**:
- `THRESHOLD_OVER_TARGET`: Reaching waypoints above objects (0.025m)
- `THRESHOLD_CLOSE_TARGET`: Approaching objects (0.01m)
- `THRESHOLD_PRECISE`: Final placement precision (0.001m)
- `THRESHOLD_PRECISE_STRICT`: Strictest precision (0.0005m)

**LLM Settings**:
- `MAX_VALIDATION_ITERATIONS`: Maximum plan validation cycles (default: 3)
- `VALIDATION_MODEL`: Model for batch validation
- `INTERACTIVE_MAX_TOKENS`: Max tokens for interactive responses

**Camera Settings**:
- `CAMERA_RESOLUTION`: Image resolution [width, height]
- `PANORAMA_FORMAT`: Image format ("JPEG" or "PNG")
- `PANORAMA_QUALITY`: JPEG quality (1-100)

## Operational Modes

The system provides three distinct modes, each designed for different use cases with different LLM integration strategies.

### 1. Batch Mode (main.py)

**Purpose**: Automated task execution from predefined scene configurations with plan validation.

**Usage**:
```bash
python main.py --scene default
python main.py --scene scene_01
```

**Features**:
- Loads scene from YAML configuration
- Captures initial panorama
- Generates task plan using LLM
- Validates plan through critique-refinement loop
- Executes validated plan sequentially
- Exits with status code (0=success, 1=failure)

**When to Use**:
- Automated testing of robot capabilities
- Benchmarking task completion
- Reproducible scenario execution
- CI/CD pipeline integration

**Control Flow**:
1. Parse command-line arguments for scene selection
2. Initialize PyBullet in GUI mode (direct connection)
3. Load scene objects from YAML
4. Stabilize robot and capture panorama
5. **LLM Planning Phase**: Generate initial plan with vision
6. **LLM Validation Phase**: Critique and refine plan (up to MAX_VALIDATION_ITERATIONS)
7. **Execution Phase**: Execute validated plan step-by-step
8. Exit with status code

### 2. Console Interactive Mode (main_console.py)

**Purpose**: Terminal-based chat interface with continuous physics simulation and rich formatting.

**Usage**:
```bash
python main_console.py --scene default
```

**Features**:
- Rich terminal UI with colored output and tables
- Continuous physics simulation while waiting for input
- Real-time token tracking and statistics
- Verbose mode for detailed tool execution visibility
- Non-blocking input handling (simulation runs while typing)
- Session summary on exit

**Special Commands**:
- `/help` - Show available commands
- `/status` - Display scene info, objects, gripper state, tokens
- `/quit` - Exit the console
- `/clear` - Clear conversation history
- `/verbose` - Toggle verbose mode (show tool details)

**When to Use**:
- Development and debugging
- SSH/remote server access (no GUI browser required)
- Lightweight interactive control
- Automated scripts with human interaction

**Control Flow**:
1. Initialize PyBullet in GUI mode
2. Load scene and stabilize robot
3. Initialize InteractiveLLMController
4. Enter chat loop:
   - Wait for user input (non-blocking, simulation continues)
   - Parse special commands or send to LLM
   - **LLM Tool Calling**: Claude uses native tools to execute actions
   - Display response with rich formatting
   - Update token counters
5. Exit on `/quit` or Ctrl+D

### 3. Streamlit Interactive Mode (main_interactive.py)

**Purpose**: Web-based chat interface with visual feedback and image display.

**Usage**:
```bash
streamlit run main_interactive.py
```

**Features**:
- Modern web UI with chat interface
- Sidebar with real-time scene information
- Inline panorama image display
- Expandable tool execution details
- Token usage tracking per message
- Chat history persistence (session-based)
- Example commands in sidebar

**When to Use**:
- User demonstrations
- Teaching and presentations
- Visual debugging (inline images)
- User-friendly interaction (non-technical users)

**Control Flow**:
1. Initialize Streamlit app and page configuration
2. Initialize PyBullet simulation (cached resource)
3. Load scene and stabilize robot (cached)
4. Initialize InteractiveLLMController (cached)
5. Render chat interface:
   - Display chat history
   - Render sidebar (objects, gripper state, token usage)
   - Accept user input via chat input widget
6. On user message:
   - **LLM Tool Calling**: Claude uses native tools to execute actions
   - Display assistant response
   - Show tool execution details in expanders
   - Render panorama images inline
   - Update token counters
   - Re-render page (Streamlit rerun)

## LLM Integration Details

This section explains where, how, and why LLM calls are made in each operational mode, along with the complete workflow.

### Batch Mode LLM Workflow

**Location**: [main.py](main.py) lines 95-182, implemented in [src/llm_validator.py](src/llm_validator.py) and [src/llm_controller.py](src/llm_controller.py)

**Three-Phase LLM Process**:

#### Phase 1: Plan Generation
**Where**: `LLMValidator._generate_initial_plan()` ([src/llm_validator.py](src/llm_validator.py):133-216)

**What happens**:
1. Constructs system prompt from `planning_system_prompt.txt` with object positions
2. Constructs user prompt from `planning_user_prompt.txt` with task description
3. Encodes panorama image as base64
4. Calls Claude API with:
   - System prompt (describes available commands and constraints)
   - Panorama image (vision input, cached)
   - Task description (what to accomplish)
5. Parses JSON response into plan structure:
   ```json
   {
     "reasoning": "...",
     "commands": [
       {"action": "pick_up", "object": "blue_cube"},
       {"action": "place_on", "object": "red_cube"}
     ]
   }
   ```

**Why**: The LLM needs visual understanding of the scene layout and object positions to generate a feasible manipulation plan.

#### Phase 2: Plan Critique
**Where**: `LLMValidator._critique_plan()` ([src/llm_validator.py](src/llm_validator.py):218-307)

**What happens**:
1. Takes generated plan as input
2. Constructs review prompt from `review_system_prompt.txt` (cached)
3. Sends plan + panorama + task to Claude for review
4. Claude critiques plan for:
   - Physical feasibility (reachability, stability)
   - Command correctness (valid syntax, object names)
   - Task completion (does it achieve the goal?)
5. Returns critique structure:
   ```json
   {
     "is_valid": false,
     "critique": "The plan attempts to stack objects that are too far apart...",
     "suggestions": [
       "Consider moving objects closer first",
       "Add intermediate placement steps"
     ]
   }
   ```

**Why**: Self-review catches issues before execution, reducing failed attempts and improving safety.

#### Phase 3: Plan Refinement (if needed)
**Where**: `LLMValidator._refine_plan()` ([src/llm_validator.py](src/llm_validator.py):309-402)

**What happens**:
1. Takes original plan + critique + suggestions
2. Constructs refinement prompt from `refinement_system_prompt.txt`
3. Sends all context to Claude with instruction to improve the plan
4. Claude generates revised plan addressing critique points
5. New plan goes back to Phase 2 (critique) for validation
6. Loop continues up to `MAX_VALIDATION_ITERATIONS` times

**Why**: Iterative refinement allows the LLM to learn from its mistakes and converge on a valid solution.

#### Validation Loop
**Where**: `LLMValidator.get_validated_plan()` ([src/llm_validator.py](src/llm_validator.py):404-466)

**Complete Workflow**:
```
1. Generate initial plan (LLM call #1)
   ↓
2. Critique plan (LLM call #2)
   ↓
3. Is plan valid?
   Yes → Return plan, execute
   No → Continue
   ↓
4. Refine plan based on critique (LLM call #3)
   ↓
5. Critique refined plan (LLM call #4)
   ↓
6. Is plan valid OR max iterations reached?
   Yes → Return plan (may be invalid if max reached)
   No → Go to step 4
```

**Typical LLM call count**: 2-8 calls (1 planning + 1-7 critique/refinement cycles)

**Exit conditions**:
- Plan passes validation (`is_valid: true`)
- Max iterations reached (returns last plan with failure flag)

#### Execution Phase
**Where**: `LLMController.execute_plan()` ([src/llm_controller.py](src/llm_controller.py):64-121)

**What happens**:
- **NO LLM CALLS** - pure deterministic execution
- Parses validated plan JSON
- Executes commands sequentially:
  - `pick_up` → `RobotController.pick_up()`
  - `place` → `RobotController.place()`
  - `place_on` → `RobotController.place_on()`
  - `rotate_gripper_90` → `RobotController.rotate_orientation_90()`
  - `reset_gripper_orientation` → `RobotController.reset_orientation()`
- Logs progress and errors

**Why**: Separation of planning (LLM) and execution (deterministic) ensures reproducibility and safety.

### Interactive Mode LLM Workflow

**Location**: [main_console.py](main_console.py) and [main_interactive.py](main_interactive.py), implemented in [src/interactive_llm_controller.py](src/interactive_llm_controller.py)

**Single-Phase Process with Tool Calling**:

#### Tool-Based Interaction
**Where**: `InteractiveLLMController.handle_message()` ([src/interactive_llm_controller.py](src/interactive_llm_controller.py):466-651)

**What happens**:
1. User sends natural language message
2. System prompt loaded from `interactive_system_prompt.txt` (cached)
3. Tool definitions registered (14 tools, cached):
   - Query tools: `get_gripper_position`, `get_object_position`, `get_all_objects`, `get_panorama`
   - Movement tools: `move_gripper`, `move_gripper_smooth`, `open_gripper`, `close_gripper`
   - Manipulation tools: `pick_up_object`, `place_object`, `place_on_object`
   - Orientation tools: `rotate_gripper_90`, `reset_gripper_orientation`
4. Call Claude API with:
   - Cached system prompt
   - Cached tool definitions
   - Conversation history (all previous messages)
   - New user message
5. Claude responds with text and/or tool calls
6. **Tool execution loop**:
   ```
   While response contains tool calls:
     - Execute each tool via InteractiveLLMController.execute_tool()
     - Collect tool results
     - Send tool results back to Claude
     - Claude may use more tools or respond with text
   ```
7. Return final text response + tool results + any panorama images

**Example Flow**:
```
User: "Stack the blue cube on the red cube"
  ↓
LLM Call #1:
  - Thinks: "I need to know where these objects are"
  - Uses tool: get_object_position("blue_cube")
  - Uses tool: get_object_position("red_cube")
  ↓
Tool Results: blue at [0.2, 0.3, 0.05], red at [0.4, 0.3, 0.05]
  ↓
LLM Call #2:
  - Thinks: "I'll pick up blue and place it on red"
  - Uses tool: pick_up_object("blue_cube")
  ↓
Tool Result: Success
  ↓
LLM Call #3:
  - Uses tool: place_on_object("red_cube")
  ↓
Tool Result: Success
  ↓
LLM Call #4 (final):
  - Text: "I've successfully stacked the blue cube on top of the red cube."
```

**Total LLM calls**: Variable (1-10+ depending on task complexity and tool chaining)

**Why this approach**:
- **Flexibility**: Claude decides which tools to use and in what order
- **Conversational**: Can ask clarifying questions or explain actions
- **Adaptive**: Can respond to errors and try alternative approaches
- **Transparent**: Tool calls are visible to user (in verbose mode)

### Tool Execution
**Where**: `InteractiveLLMController.execute_tool()` ([src/interactive_llm_controller.py](src/interactive_llm_controller.py):271-303)

**What happens**:
1. Receives tool name and parameters from Claude
2. Validates tool exists
3. Calls internal `_execute_tool_impl()` with parameters
4. Catches any exceptions and returns error result
5. Logs tool call and result

**Why**: Provides safe, structured interface between LLM and robot.

### Token Usage and Caching

Both modes use **Anthropic Prompt Caching** to reduce costs:

**Batch Mode Caching**:
- Panorama image (cached across critique/refinement cycles)
- Review system prompt (cached across critique cycles)

**Interactive Mode Caching**:
- System prompt (cached across entire conversation)
- Tool definitions (cached across entire conversation)

**Cache Behavior**:
- First call: Creates cache (slightly higher cost)
- Subsequent calls: Reads from cache (90% cost reduction)
- Cache TTL: 5 minutes of inactivity
- Cache key: Content hash (automatic)

**Token Logging**:
- Input tokens (prompt)
- Output tokens (response)
- Cache creation tokens (first time content cached)
- Cache read tokens (reusing cached content)

## Scenes

Scene configurations are stored as YAML files in [scenes/](scenes/) directory.

### Scene File Format

```yaml
metadata:
  name: "Scene Name"
  description: "Brief description"

objects:
  - name: "blue_cube"
    type: "cube"
    position: [0.3, 0.2, 0.05]
    color: [0.0, 0.0, 1.0, 1.0]  # RGBA
    scale: 1.0

task:
  description: "Stack all cubes in a tower"
```

### Available Scenes

- **default.yaml**: Basic scene with multiple cubes
- **scene_01.yaml**: Simple stacking task
- **scene_02.yaml**: Complex arrangement
- **scene_03.yaml**: Multiple object manipulation
- **scene_04.yaml**: Advanced stacking
- **scene_05.yaml**: Challenging spatial reasoning

### Creating Custom Scenes

1. Create new YAML file in `scenes/` directory
2. Follow format in [scenes/README.md](scenes/README.md)
3. Run with: `python main.py --scene my_scene`

**Important Scene Design Tips**:
- Object names should be descriptive (include color for clarity)
- Robot workspace: x: [-0.5, 0.7], y: [-0.7, 0.7], z: [0, 0.8]
- Ground level: z = 0.05 for objects
- Avoid overlapping object positions
- Keep objects within reachable workspace

## Prompts

LLM prompt templates are stored in [prompts/](prompts/) directory.

### Batch Mode Prompts

**Planning Phase**:
- `planning_system_prompt.txt` - Describes task planning constraints and available commands
- `planning_user_prompt.txt` - Template for task description

**Review Phase**:
- `review_system_prompt.txt` - Instructions for critiquing plans
- `review_user_prompt.txt` - Template for review requests

**Refinement Phase**:
- `refinement_system_prompt.txt` - Instructions for improving plans based on critique
- `refinement_user_prompt.txt` - Template for refinement requests

### Interactive Mode Prompts

- `interactive_system_prompt.txt` - System prompt defining assistant behavior and tool usage guidelines

### Prompt Placeholders

Prompts use these placeholders (filled at runtime):
- `{OBJECTS_LIST}` - Bullet list of object names
- `{OBJECTS_INFO}` - Detailed object positions and dimensions
- `{TASK_DESCRIPTION}` - Natural language task description
- `{PLAN}` - JSON plan structure
- `{CRITIQUE}` - Critique text and suggestions
- `{ORIGINAL_PLAN}` - Previous plan version

### Customizing Prompts

1. Edit prompt files in `prompts/` directory
2. Maintain placeholder format for dynamic content
3. Test with various scenes to ensure robustness
4. Consider adding examples in prompts for better performance

## Logging

Comprehensive logging system tracks all simulation activity and LLM interactions.

### Log Directory Structure

```
logs/
├── app_YYYYMMDD_HHMMSS.log          # Main application log
├── api_calls_YYYYMMDD_HHMMSS.log   # Detailed API call logs (if enabled)
└── ...
```

### Log Levels

**Application Logs** (`app_*.log`):
- Scene loading and object initialization
- Robot movement and command execution
- LLM plan generation and validation
- Tool execution results
- Errors and warnings
- Simulation timing

**API Call Logs** (`api_calls_*.log`):
- Full LLM request payloads (system prompt, user prompt, tools)
- Full LLM response payloads
- Token usage details (input, output, cache metrics)
- Request/response timestamps
- Conversation history (interactive mode)

**Console Output**:
- High-level progress messages
- Command execution status
- Validation results
- Token usage summaries
- Error messages

### Logging Configuration

Edit [src/config.py](src/config.py):
```python
LOGS_FOLDER = 'logs'
LOG_SESSION_NAME = f'app_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
LOG_API_CALLS = True  # Enable detailed API logging
```

### Logger Methods

From [src/logger.py](src/logger.py):
- `console_info()` - Info messages to console
- `console_success()` - Success messages (green)
- `console_warning()` - Warning messages (yellow)
- `console_error()` - Error messages (red)
- `log_llm_request()` - Log LLM API request
- `log_llm_response()` - Log LLM API response
- `log_tool_call()` - Log tool execution (interactive mode)
- `log_interactive_message()` - Log chat messages (interactive mode)

### Viewing Logs

**Real-time monitoring**:
```bash
tail -f logs/app_*.log
```

**Search for errors**:
```bash
grep ERROR logs/app_*.log
```

**Token usage summary**:
```bash
grep "tokens" logs/app_*.log
```

## Robot Commands

### Available Commands

#### Basic Movement
- `move_to_target(position, threshold)` - Move end effector to target position using IK
- `move_to_target_smooth(position, threshold)` - Smooth linear interpolated movement

#### Gripper Control
- `open_gripper()` - Open gripper
- `close_gripper()` - Close gripper

#### Orientation
- `rotate_orientation_90()` - Rotate gripper 90° (avoid hitting objects with wide part along Y-axis)
- `reset_orientation()` - Reset to default downward orientation

#### High-Level Manipulation
- `pick_up(object_name)` - Complete pick-up sequence (approach, grasp, lift)
- `place(position)` - Place held object at absolute position
- `place_on(object_name)` - Place held object on top of target object

### Movement Thresholds

From [src/config.py](src/config.py):
- `THRESHOLD_OVER_TARGET = 0.025` - Reaching waypoints above objects
- `THRESHOLD_CLOSE_TARGET = 0.01` - Approaching objects
- `THRESHOLD_PRECISE = 0.001` - Final placement precision
- `THRESHOLD_PRECISE_STRICT = 0.0005` - Strictest precision

### Pick-and-Place Workflow

**Pick-up sequence**:
1. Open gripper
2. Move to position above object (+ offset)
3. Descend to grasp height
4. Close gripper
5. Lift to safe height

**Place sequence**:
1. Move to position above target (+ offset)
2. Descend to placement height
3. Open gripper
4. Retract upward

**Place-on sequence**:
1. Query target object dimensions
2. Calculate placement position on top
3. Execute place sequence at calculated position

## Project Structure

```
milab_bead_j0p7mf_2/
├── main.py                      # Batch mode entry point
├── main_console.py              # Console interactive mode entry point
├── main_interactive.py          # Streamlit interactive mode entry point
├── requirements.txt             # Python dependencies
├── .env                         # Environment variables (not in git)
├── .gitignore                   # Git ignore rules
├── CLAUDE.md                    # Project instructions for Claude Code
├── README.md                    # This file
│
├── src/                         # Source code modules
│   ├── config.py                # Centralized configuration
│   ├── robot_controller.py      # Robot control and IK
│   ├── object_manager.py        # Object registry and queries
│   ├── camera_manager.py        # Panorama capture
│   ├── simulation_state.py      # State tracking
│   ├── llm_controller.py        # Batch mode plan execution
│   ├── llm_validator.py         # Batch mode plan validation
│   ├── interactive_llm_controller.py  # Interactive tool calling
│   ├── logger.py                # Logging utilities
│   └── scene_loader.py          # YAML scene loading
│
├── scenes/                      # Scene configurations
│   ├── README.md                # Scene format documentation
│   ├── default.yaml             # Default scene
│   └── scene_*.yaml             # Additional scenes
│
├── prompts/                     # LLM prompt templates
│   ├── planning_system_prompt.txt
│   ├── planning_user_prompt.txt
│   ├── review_system_prompt.txt
│   ├── review_user_prompt.txt
│   ├── refinement_system_prompt.txt
│   ├── refinement_user_prompt.txt
│   └── interactive_system_prompt.txt
│
├── images/                      # Generated panorama images
│   └── panorama_*.jpg
│
└── logs/                        # Simulation and API logs
    ├── app_*.log
    └── api_calls_*.log
```

## Development

### Adding New Object Types

1. Extend [src/object_manager.py](src/object_manager.py) with `load_<type>()` method
2. Update [src/scene_loader.py](src/scene_loader.py) validation schema
3. Modify scene loading loop in [main.py](main.py):68-75

### Modifying LLM Behavior

**Batch mode**: Edit prompt files in [prompts/](prompts/) directory
**Interactive mode**: Edit `interactive_system_prompt.txt` or modify tool definitions in [src/interactive_llm_controller.py](src/interactive_llm_controller.py):81-269

### Adjusting Movement Precision

Edit threshold constants in [src/config.py](src/config.py):
- `THRESHOLD_OVER_TARGET` - Reaching waypoints
- `THRESHOLD_CLOSE_TARGET` - Approaching objects
- `THRESHOLD_PRECISE` - Final placement
- `THRESHOLD_PRECISE_STRICT` - Strictest precision

### Testing

Run specific scenes to test modifications:
```bash
python main.py --scene test_scene
```

Monitor logs for debugging:
```bash
tail -f logs/app_*.log
```

### Platform-Specific Notes

**macOS Compatibility**:
- PyBullet shared memory connection can hang on macOS
- All modes use direct GUI connection (`mode='gui'`)
- Debug visualizer configuration skipped in headless mode
- See [src/robot_controller.py](src/robot_controller.py):16-66 for initialization logic

## License

[Add your license information here]

## Acknowledgments

- Built with [PyBullet](https://pybullet.org/) physics simulation
- Powered by [Anthropic Claude](https://www.anthropic.com/claude) API
- Franka Panda robot model from PyBullet examples
