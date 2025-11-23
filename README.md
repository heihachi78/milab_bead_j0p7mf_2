# LLM-Controlled Robot Simulation

An intelligent robotic simulation system that uses Claude AI to plan and execute pick-and-place tasks with a Franka Panda robotic arm in PyBullet. The system combines visual scene understanding through multi-camera panoramas with natural language task descriptions to autonomously generate, validate, and execute manipulation plans.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
- [Requirements](#requirements)
- [Usage](#usage)
  - [Batch Mode (Automated Execution)](#batch-mode-automated-execution)
  - [Interactive Mode (Conversational Control)](#interactive-mode-conversational-control)
- [Scene Configuration](#scene-configuration)
- [LLM Integration Deep Dive](#llm-integration-deep-dive)
  - [Batch Mode LLM Workflow](#batch-mode-llm-workflow)
  - [Interactive Mode LLM Workflow](#interactive-mode-llm-workflow)
  - [Prompt Engineering](#prompt-engineering)
  - [Token Optimization](#token-optimization)
- [Architecture](#architecture)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)

## Overview

This project implements three execution modes for LLM-controlled robotic manipulation:

1. **Batch Mode** ([main.py](main.py)): Fully automated task execution with a sophisticated plan validation pipeline. The system uses a critique-refinement loop to iteratively improve task plans before execution.

2. **Interactive Mode** ([main_interactive.py](main_interactive.py)): Real-time conversational control through a web-based chat interface. Uses Claude's native tool-calling API to enable natural language robot commands.

3. **Visual Server Mode** ([main_visual.py](main_visual.py)): PyBullet GUI server that enables visualization for interactive mode on macOS. Required due to macOS threading restrictions.

All modes leverage Claude's multimodal capabilities, processing panoramic images alongside natural language to understand the 3D scene and plan collision-free manipulation strategies.

## Features

- **Visual Scene Understanding**: Multi-camera panorama system (5 viewpoints: front, right, back, left, top)
- **LLM-Driven Planning**: Autonomous generation of pick-and-place task plans from natural language
- **Iterative Plan Validation**: Self-critique and refinement loop ensures plan feasibility
- **Collision Avoidance**: Gripper orientation planning to prevent collisions with nearby objects
- **YAML-Based Scene Configuration**: Easy-to-define object layouts and task descriptions
- **Dual Execution Modes**: Both batch processing and interactive conversational control
- **Comprehensive Logging**: Detailed logs of LLM interactions, robot operations, and token usage
- **Prompt Caching**: Efficient API usage through Anthropic's prompt caching for repeated elements

## Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Create Virtual Environment

**Using venv (standard):**
```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

**Using conda (recommended for macOS):**
```bash
conda create -n pybullet_env python=3.11
conda activate pybullet_env
```

### 3. Install Dependencies

**Using pip:**
```bash
pip install -r requirements.txt
```

**Using conda (macOS - recommended):**
```bash
conda install -c conda-forge pybullet anthropic numpy pillow pyyaml python-dotenv streamlit scipy opencv jsonschema pytest
```

**Note for macOS users:** PyBullet installation via conda is more reliable on macOS due to native dependencies.

Required packages:
- `pybullet>=3.2.5` - Physics simulation engine
- `anthropic>=0.25.0` - Claude API client
- `numpy>=1.24.0` - Numerical computations
- `pillow>=10.0.0` - Image processing
- `pyyaml>=6.0` - Scene configuration loading
- `python-dotenv>=1.0.0` - Environment variable management
- `streamlit>=1.28.0` - Interactive web interface

Optional packages:
- `scipy>=1.11.0` - Advanced scientific computing
- `opencv-python>=4.8.0` - Computer vision utilities
- `jsonschema>=4.19.0` - Schema validation
- `pytest>=7.4.0` - Testing framework

### 4. Configure Environment Variables

Create a `.env` file in the project root:

```bash
# .env file
ANTHROPIC_API_KEY=your_api_key_here
ANTHROPIC_MODEL=claude-3-5-haiku-20241022
```

**How to obtain an API key:**

1. Go to [console.anthropic.com](https://console.anthropic.com/)
2. Sign up or log in to your Anthropic account
3. Navigate to API Keys section
4. Generate a new API key
5. Copy the key to your `.env` file

**Available models:**
- `claude-3-5-haiku-20241022` - Fast, cost-efficient (recommended for development)
- `claude-3-5-sonnet-20241022` - Balanced performance
- `claude-sonnet-4-5-20250929` - Most capable, highest cost
- `claude-opus-4-20250514` - Maximum capability

**Important:** Never commit your `.env` file to version control. The `.gitignore` file should already exclude it.

## Requirements

### System Requirements

- **Python**: 3.11 or higher
- **Operating System**: Linux, macOS, or Windows
- **Display**: GUI support for PyBullet visualization
- **Memory**: Minimum 4GB RAM recommended

### Platform-Specific Notes

**macOS:**
- Interactive mode with GUI visualization requires running two separate processes (see [Interactive Mode](#interactive-mode-conversational-control))
- This is due to macOS threading restrictions: NSWindow/AppKit must run on the main thread
- Batch mode works normally with direct GUI access
- Alternative: Use headless mode with panorama captures for visual feedback

**Linux/Windows:**
- All modes work without additional setup
- Interactive mode can create GUI windows directly from Streamlit
- No multi-process architecture required (but supported)

### API Requirements

- **Anthropic API Key**: Required for all LLM functionality
- **API Credits**: Ensure sufficient credits for your usage
  - Batch mode: ~50-200 tokens per validation cycle
  - Interactive mode: ~100-500 tokens per message depending on tool usage
  - Prompt caching significantly reduces costs for repeated elements

### Network Requirements

- Stable internet connection for Claude API calls
- No special firewall configuration needed (uses HTTPS on port 443)

## Usage

### Batch Mode (Automated Execution)

Batch mode executes a predefined task from a scene configuration file using a fully automated validation pipeline.

**Basic usage:**

```bash
python main.py
```

**With custom scene:**

```bash
python main.py --scene default
python main.py --scene scene_1
python main.py --scene example_stacking
```

**What happens during execution:**

1. **Initialization**: Loads scene configuration, initializes PyBullet simulation
2. **Object Loading**: Spawns objects from scene YAML file into the simulation
3. **Stabilization**: Allows physics to settle, positions robot at home pose
4. **Panorama Capture**: Takes 5-viewpoint panorama for visual scene understanding
5. **Plan Generation**: LLM generates initial task plan from task description + panorama
6. **Validation Loop**: Iterative critique-refinement cycle (up to 3 iterations by default)
7. **Execution**: Robot executes the validated plan step-by-step
8. **Logging**: Saves detailed logs to `logs/` directory with timestamps

**Command-line arguments:**

- `--scene SCENE_NAME`: Specify which scene configuration to load (default: `default`)
  - Scene files are located in `scenes/` directory
  - Use filename without `.yaml` extension

**Output files:**

- **Logs**: `logs/session_YYYYMMDD_HHMMSS.log` - Complete execution log
- **Panoramas**: `images/panorama_NNN_operation_name.jpg` - Visual snapshots after each operation
- **Console output**: Real-time progress updates and status messages

### Interactive Mode (Conversational Control)

Interactive mode provides a web-based chat interface for natural language robot control.

#### macOS Users: Visual Server Setup (Required for GUI Visualization)

**Why this is needed:** macOS requires all GUI window creation (NSWindow/AppKit) to happen on the main thread. Since Streamlit runs Python code in a background thread, PyBullet cannot create GUI windows directly. The solution is to run a separate PyBullet GUI server process.

**Terminal 1 - Start PyBullet GUI server:**

```bash
python main_visual.py
python main_visual.py --scene example_stacking  # With custom scene
```

This will:
- Open the PyBullet GUI window (visualization)
- Run as a server waiting for connections
- Display scene with robot and objects

**Terminal 2 - Start Streamlit interface:**

```bash
streamlit run main_interactive.py
streamlit run main_interactive.py -- --scene example_stacking  # Must match Terminal 1 scene
```

**Important:** Both terminals must use the **same `--scene` argument** to load matching configurations.

The Streamlit interface will automatically detect and connect to the GUI server via shared memory, allowing you to control the robot through the web chat while seeing real-time movements in the PyBullet window.

#### Linux/Windows Users: Direct Launch

On Linux and Windows, you can run interactive mode directly without the visual server:

```bash
streamlit run main_interactive.py
streamlit run main_interactive.py -- --scene default
```

The PyBullet GUI will open automatically alongside the web interface.

#### Headless Mode (All Platforms)

If no visual server is running (or if you prefer headless operation), Streamlit will automatically fall back to headless mode with a helpful message:

```
No PyBullet GUI server found, using headless mode
Tip: Run 'python main_visual.py' in another terminal for visualization
```

You can still control the robot and request panorama captures through the chat interface to see the scene.

**Note:** The double dash `--` separates Streamlit arguments from script arguments.

**Accessing the interface:**

1. After running the command, Streamlit will output a URL (typically `http://localhost:8501`)
2. Open the URL in your web browser
3. Wait for initialization (connection status shown in console)
4. Start chatting with the robot

**Example commands:**

```
"What objects are in the scene?"
"Pick up the blue cube"
"Show me the scene"
"Where is the red cube?"
"Move the gripper to position [0.3, 0.4, 0.2]"
"Stack the red cube on top of the blue cube"
"Open the gripper"
"Place the object I'm holding at [0.2, 0.2, 0.05]"
```

**Interface features:**

- **Chat History**: Full conversation log with user and assistant messages
- **Tool Execution Display**: Expandable sections showing tool calls and results
- **Panorama Display**: Inline panorama images when LLM requests visual info
- **Token Usage Tracking**: Real-time display of input/output tokens used
- **Scene Information Sidebar**: Live object positions and gripper state
- **Example Commands**: Quick reference for common operations

**Available tools in interactive mode:**

The LLM has access to 12 tools organized into categories:

1. **Query Tools** (Scene Information):
   - `get_gripper_position`: Get current gripper coordinates
   - `get_gripper_state`: Check if gripper is open or closed
   - `get_object_position`: Query specific object location
   - `get_all_objects`: List all objects with positions and dimensions
   - `get_panorama`: Capture and view current scene panorama

2. **Movement Tools** (Low-Level Control):
   - `move_gripper`: IK-based movement to target position
   - `move_gripper_smooth`: Smooth linear interpolation movement

3. **Gripper Tools** (Gripper Control):
   - `open_gripper`: Open gripper fingers
   - `close_gripper`: Close gripper to grasp

4. **Orientation Tools** (Gripper Rotation):
   - `rotate_gripper_90`: Rotate gripper 90° for collision avoidance
   - `reset_gripper_orientation`: Return to default orientation

5. **High-Level Tools** (Complex Operations):
   - `pick_up_object`: Complete pick sequence (approach → grasp → lift)
   - `place_object`: Place at absolute coordinates
   - `place_on_object`: Place on top of another object

## Scene Configuration

Scenes are defined in YAML files located in the `scenes/` directory. Each scene contains object definitions and an optional task description.

### Scene File Structure

```yaml
metadata:
  name: "Scene Name"
  description: "Brief description of the scene"

objects:
  - name: "red_cube"
    type: "cube"
    position: [0.3, 0.4, 0.05]  # [x, y, z] in meters
    color: [1, 0, 0, 1]          # [r, g, b, a] (0.0-1.0)
    scale: 1.0

  - name: "blue_cube"
    type: "cube"
    position: [0.3, 0.2, 0.05]
    color: [0, 0, 1, 1]
    scale: 1.0

task:
  description: "Stack the red cube on top of the blue cube"
```

### Field Descriptions

**Metadata:**
- `name`: Human-readable scene name (displayed in logs and UI)
- `description`: Brief explanation of the scene (optional)

**Objects:**
- `name`: Unique identifier for the object (used in commands)
- `type`: Object type (currently supports `"cube"`)
- `position`: `[x, y, z]` coordinates in meters
  - `x, y`: Horizontal position (workspace center ≈ [0.3, 0.3])
  - `z`: Vertical position (ground level = 0, typical cube center = 0.05)
- `color`: `[r, g, b, a]` RGBA values (range 0.0-1.0)
  - Red: `[1, 0, 0, 1]`
  - Green: `[0, 1, 0, 1]`
  - Blue: `[0, 0, 1, 1]`
  - Yellow: `[1, 1, 0, 1]`
  - Purple: `[0.5, 0, 0.5, 1]`
- `scale`: Size multiplier (1.0 = standard cube size ≈ 0.05m)

**Task:**
- `description`: Natural language task description (used in batch mode)
  - Be specific about object names and desired final configuration
  - Example: "Create a stack in this order from bottom to top: blue cube, yellow cube, red cube"

### Creating Custom Scenes

1. Create a new YAML file in `scenes/` directory:

```bash
nano scenes/my_custom_scene.yaml
```

2. Define your scene following the structure above

3. Run with your scene:

```bash
python main.py --scene my_custom_scene
streamlit run main_interactive.py -- --scene my_custom_scene
```

### Scene Design Guidelines

**Robot workspace:**
- Safe X range: 0.0 to 0.6 meters
- Safe Y range: -0.3 to 0.6 meters
- Safe Z range: 0.0 to 0.8 meters

**Object placement tips:**
- Leave at least 0.1m clearance between objects for gripper access
- Place objects within robot reach (distance from [0, 0, 0] < 0.8m)
- Consider gripper orientation when placing objects close together
- Start objects at z=0.05 for standard cube size (center of 0.1m cube)

**Task descriptions:**
- Be explicit about object names
- Specify order for stacking operations
- Avoid ambiguous language
- Example good task: "Place red_cube on top of blue_cube, then place green_cube on top of red_cube"
- Example bad task: "Stack some cubes"

## LLM Integration Deep Dive

This section provides comprehensive documentation of how Claude AI is integrated into both execution modes, including API calls, prompting strategies, and workflows.

### Batch Mode LLM Workflow

Batch mode uses a sophisticated **critique-refinement loop** to iteratively improve task plans before execution. This ensures high-quality, feasible plans.

#### Workflow Stages

The validation workflow consists of three distinct LLM phases:

```
1. PLANNING → 2. REVIEW → 3. REFINEMENT
                    ↑            ↓
                    └────────────┘
                  (repeat up to MAX_VALIDATION_ITERATIONS)
```

#### 1. Planning Phase

**Purpose:** Generate initial task plan from natural language description and visual scene understanding.

**LLM Call Configuration:**
```python
model = ANTHROPIC_MODEL  # from .env or config
max_tokens = 2048
system_prompt = planning_system_prompt.txt (formatted with objects list/info)
user_message = [
    panorama_image (base64 encoded, cached),
    planning_user_prompt.txt (formatted with task)
]
```

**Prompt Templates:**
- System: [prompts/planning_system_prompt.txt](prompts/planning_system_prompt.txt)
  - Defines available objects and their positions
  - Lists available commands (pick_up, place, place_on, rotate_gripper_90, reset_gripper_orientation)
  - Provides detailed instructions on gripper orientation and collision avoidance
  - Includes examples of correct planning
- User: [prompts/planning_user_prompt.txt](prompts/planning_user_prompt.txt)
  - Contains the task description
  - References the attached panorama image

**Prompt Variables:**
- `{OBJECTS_LIST}`: Bullet list of object names
- `{OBJECTS_INFO}`: Detailed list with positions and dimensions
- `{TASK_DESCRIPTION}`: Natural language task from scene YAML

**Output Format:**
```json
{
  "reasoning": "Explanation of the approach and logic",
  "commands": [
    {"action": "pick_up", "object": "red_cube"},
    {"action": "place_on", "object": "blue_cube"}
  ]
}
```

**Example API Call:**
```python
response = client.messages.create(
    model="claude-3-5-haiku-20241022",
    max_tokens=2048,
    system=planning_system_prompt,
    messages=[{
        "role": "user",
        "content": [
            {
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": "image/jpeg",
                    "data": panorama_base64
                },
                "cache_control": {"type": "ephemeral"}
            },
            {
                "type": "text",
                "text": "Task: Stack red cube on blue cube"
            }
        ]
    }]
)
```

**Why This Design:**
- Panorama provides visual context for spatial understanding
- Object positions enable precise coordinate planning
- Reasoning field improves plan quality through chain-of-thought
- Caching panorama reduces API costs in refinement iterations

#### 2. Review Phase

**Purpose:** Critically evaluate the generated plan for feasibility, correctness, and safety.

**LLM Call Configuration:**
```python
model = ANTHROPIC_MODEL
max_tokens = 2048
system_prompt = review_system_prompt.txt (cached)
user_message = [
    panorama_image (base64 encoded, cached),
    review_user_prompt.txt (formatted with task, objects, plan)
]
```

**Prompt Templates:**
- System: [prompts/review_system_prompt.txt](prompts/review_system_prompt.txt)
  - Defines role as critical reviewer
  - Lists evaluation criteria (feasibility, safety, efficiency, correctness)
  - Emphasizes collision detection and gripper orientation validation
  - Cached for efficiency (static across iterations)
- User: [prompts/review_user_prompt.txt](prompts/review_user_prompt.txt)
  - Contains original task, object info, and plan to review
  - References panorama for visual verification

**Prompt Variables:**
- `{TASK_DESCRIPTION}`: Original task
- `{OBJECTS_INFO}`: Current object positions
- `{PLAN}`: JSON plan to review

**Output Format:**
```json
{
  "is_valid": false,
  "critique": "The plan would cause gripper collision with green_cube when picking up red_cube because...",
  "suggestions": [
    "Rotate gripper 90 degrees before picking up red_cube",
    "Pick up green_cube first to clear the area",
    "Use a different approach angle"
  ]
}
```

**Evaluation Criteria:**
1. **Feasibility**: Can the robot physically execute each command?
2. **Collision Safety**: Will gripper collide with objects during execution?
3. **Object Reachability**: Are all objects within robot workspace?
4. **Stacking Logic**: Is stacking order physically possible?
5. **Gripper Orientation**: Is orientation appropriate for dense object layouts?
6. **Task Completion**: Will the plan accomplish the stated goal?

**Why This Design:**
- Separate reviewer role improves objectivity
- Visual verification catches spatial reasoning errors
- Structured critique enables targeted refinement
- Caching static review prompt reduces costs

#### 3. Refinement Phase

**Purpose:** Improve the plan based on reviewer feedback.

**LLM Call Configuration:**
```python
model = ANTHROPIC_MODEL
max_tokens = 2048
system_prompt = refinement_system_prompt.txt (formatted with objects)
user_message = [
    panorama_image (base64 encoded, cached),
    refinement_user_prompt.txt (formatted with task, original plan, critique)
]
```

**Prompt Templates:**
- System: [prompts/refinement_system_prompt.txt](prompts/refinement_system_prompt.txt)
  - Defines role as plan improver
  - Re-states available commands and constraints
  - Emphasizes incorporating feedback
- User: [prompts/refinement_user_prompt.txt](prompts/refinement_user_prompt.txt)
  - Contains original task, objects, original plan, and critique
  - Asks for refined plan addressing all concerns

**Prompt Variables:**
- `{TASK_DESCRIPTION}`: Original task
- `{OBJECTS_INFO}`: Object positions and dimensions
- `{ORIGINAL_PLAN}`: Previous plan JSON
- `{CRITIQUE}`: Formatted critique text with suggestions

**Output Format:**
```json
{
  "reasoning": "Based on the feedback, I've addressed the collision issue by...",
  "commands": [
    {"action": "rotate_gripper_90"},
    {"action": "pick_up", "object": "red_cube"},
    {"action": "reset_gripper_orientation"},
    {"action": "place_on", "object": "blue_cube"}
  ]
}
```

**Refinement Strategies:**
- Add gripper rotation commands for collision avoidance
- Reorder pick operations to clear blocking objects
- Adjust placement positions for better clearance
- Simplify unnecessary intermediate steps

**Why This Design:**
- Targeted refinement focuses on specific issues
- Maintains context with original plan and critique
- Iterative improvement allows gradual convergence
- Preserves visual context across refinement cycles

#### Validation Loop Iteration

The system repeats review → refinement up to `MAX_VALIDATION_ITERATIONS` times (default: 3).

**Loop Logic:**
```python
for iteration in range(MAX_VALIDATION_ITERATIONS):
    critique = _critique_plan(current_plan, task, panorama)

    if critique["is_valid"]:
        return current_plan, True, critique

    if iteration < MAX_VALIDATION_ITERATIONS - 1:
        current_plan = _refine_plan(current_plan, critique, task, panorama)
    else:
        # Max iterations reached, return best effort
        return current_plan, False, critique
```

**Termination Conditions:**
1. **Success**: Reviewer marks plan as valid (`is_valid: true`)
2. **Max Iterations**: Reached iteration limit without validation
3. **Error**: JSON parsing failure or API error

**Typical Outcomes:**
- **1 iteration**: Simple, unambiguous tasks
- **2-3 iterations**: Complex tasks with collision concerns
- **Failure after 3**: Task impossible or scene misconfiguration

**Cost Optimization:**
- Panorama image cached across all calls (after first)
- Review system prompt cached (static content)
- Typical cost: ~150-300 tokens per validation cycle with caching

### Interactive Mode LLM Workflow

Interactive mode uses **native tool calling** to enable conversational robot control. This is fundamentally different from batch mode's JSON plan format.

#### Architecture

```
User Message → LLM (with tool definitions) → Tool Calls → Execute Tools → Return Results → LLM → Response
```

#### Tool Calling Mechanism

**Anthropic Tool Use API:**

Claude's tool use works through a multi-turn conversation:

1. **Initial Call**: Send user message with available tool definitions
2. **Tool Requests**: Claude responds with `tool_use` content blocks
3. **Tool Execution**: System executes requested tools
4. **Result Return**: Send tool results back to Claude
5. **Final Response**: Claude synthesizes results into user-facing message

**Tool Definition Format:**

Each tool is defined with:
- `name`: Unique tool identifier
- `description`: What the tool does (used by LLM for selection)
- `input_schema`: JSON schema for parameters

Example tool definition:
```python
{
    "name": "pick_up_object",
    "description": "High-level operation to pick up a specified object. Handles complete pick sequence: approach, open gripper, grasp, and lift.",
    "input_schema": {
        "type": "object",
        "properties": {
            "object_name": {
                "type": "string",
                "description": "Name of the object to pick up (e.g., 'blue_cube')"
            }
        },
        "required": ["object_name"]
    }
}
```

#### System Prompt

**Location:** [prompts/interactive_system_prompt.txt](prompts/interactive_system_prompt.txt)

**Purpose:** Defines the LLM's role, capabilities, and behavior guidelines for interactive mode.

**Key Elements:**
- **Role Definition**: "You are a helpful robotics assistant controlling a Franka Panda robotic arm"
- **Tool Usage Guidelines**: When and how to use each tool category
- **Spatial Reasoning**: Understanding 3D coordinates and robot workspace
- **Error Handling**: How to respond when operations fail
- **Conversation Style**: Friendly, explanatory, proactive

**System Prompt Structure:**
```
Role Definition
├── What you control (robot type, capabilities)
├── How you interact (conversational, helpful)
└── Your objectives (execute commands, provide info)

Tool Categories
├── Query Tools: When to gather information
├── Movement Tools: Low-level vs high-level
├── Gripper Tools: Open/close timing
└── High-Level Tools: Complex operation orchestration

Guidelines
├── Always check object positions before manipulation
├── Use get_panorama for visual understanding when needed
├── Explain actions as you perform them
├── Handle errors gracefully with alternatives
└── Maintain safety (collision avoidance, workspace bounds)
```

#### Conversation Flow Example

**User:** "Pick up the blue cube"

**1. LLM Initial Response:**
```json
{
  "role": "assistant",
  "content": [
    {
      "type": "text",
      "text": "I'll pick up the blue cube for you. Let me first check its position."
    },
    {
      "type": "tool_use",
      "id": "toolu_01234",
      "name": "get_object_position",
      "input": {"object_name": "blue_cube"}
    }
  ]
}
```

**2. System Executes Tool:**
```python
result = object_manager.get_object_center_position("blue_cube")
# Returns: [0.3, 0.2, 0.05]
```

**3. Tool Result Sent Back:**
```json
{
  "role": "user",
  "content": [
    {
      "type": "tool_result",
      "tool_use_id": "toolu_01234",
      "content": {
        "success": true,
        "object": "blue_cube",
        "position": [0.3, 0.2, 0.05],
        "message": "blue_cube position: [0.3000, 0.2000, 0.0500] meters"
      }
    }
  ]
}
```

**4. LLM Follow-Up:**
```json
{
  "role": "assistant",
  "content": [
    {
      "type": "text",
      "text": "The blue cube is at position [0.30, 0.20, 0.05]. Now I'll pick it up."
    },
    {
      "type": "tool_use",
      "id": "toolu_56789",
      "name": "pick_up_object",
      "input": {"object_name": "blue_cube"}
    }
  ]
}
```

**5. Execution and Final Response:**
```python
robot_controller.pick_up("blue_cube")  # Executes full pick sequence
# Returns: {"success": true, "message": "Successfully picked up blue_cube"}
```

```json
{
  "role": "assistant",
  "content": [
    {
      "type": "text",
      "text": "Done! I've successfully picked up the blue cube and it's now held by the gripper."
    }
  ]
}
```

#### Tool Categories and Usage Patterns

**1. Query Tools (Information Gathering)**

Used when LLM needs to understand current state:

```python
# Query single object
get_object_position(object_name="red_cube")
# Returns: {"position": [x, y, z], "message": "..."}

# Query all objects (scene overview)
get_all_objects()
# Returns: [{"name": "red_cube", "position": [...], "dimensions": [...]}, ...]

# Check gripper state
get_gripper_state()
# Returns: {"state": "open"|"closed", "position": 0.04}

# Get gripper position
get_gripper_position()
# Returns: {"position": [x, y, z]}

# Visual understanding
get_panorama()
# Returns: {"success": true, "panorama_base64": "...", "message": "..."}
```

**When LLM Uses These:**
- Before manipulation: Check object locations
- After operations: Verify state changes
- User asks "where is...": Direct position query
- User asks "what's in the scene": Get all objects
- Visual ambiguity: Request panorama

**2. Movement Tools (Low-Level Control)**

Direct gripper positioning:

```python
# Standard IK movement (faster, less smooth)
move_gripper(x=0.3, y=0.4, z=0.2)

# Smooth linear movement (slower, precise)
move_gripper_smooth(x=0.3, y=0.4, z=0.15)
```

**When LLM Uses These:**
- User explicitly requests position: "Move to [x, y, z]"
- Fine positioning needed: Small adjustments
- Rare in practice: High-level tools preferred

**3. Gripper Tools (Gripper Control)**

```python
open_gripper()   # Open fingers
close_gripper()  # Close to grasp
```

**When LLM Uses These:**
- Manual control: User says "open gripper"
- Low-level sequences: Custom pick operations
- Rare in practice: High-level tools handle automatically

**4. Orientation Tools (Collision Avoidance)**

```python
rotate_gripper_90()         # Rotate for perpendicular approach
reset_gripper_orientation() # Return to default
```

**When LLM Uses These:**
- Dense object layouts: Avoid collisions during approach
- User requests: "Rotate the gripper"
- Strategic repositioning: Before pick operations in tight spaces

**5. High-Level Tools (Complex Operations)**

Most commonly used for manipulation:

```python
# Complete pick sequence
pick_up_object(object_name="blue_cube")
# Internally: approach → open → grasp → lift → capture panorama

# Place at coordinates
place_object(x=0.2, y=0.2, z=0.05)
# Internally: approach → descend → release → retract

# Place on another object
place_on_object(target_object="red_cube")
# Internally: calculate top position → approach → descend → release → retract
```

**When LLM Uses These:**
- User manipulation requests: "Pick up...", "Place...", "Stack..."
- Preferred over low-level: Handles full sequences
- Automatic safety: Built-in collision avoidance
- Most common: 80%+ of manipulation commands use these

#### Multi-Step Reasoning

**Complex Task Example:** "Stack the red cube on the blue cube"

**LLM's Internal Reasoning:**
1. Break down into subtasks: pick red cube, place on blue cube
2. Gather information: Where is red cube? Where is blue cube?
3. Plan tool sequence: get_object_position × 2, pick_up_object, place_on_object
4. Execute step-by-step with user updates

**Actual Tool Call Sequence:**
```python
1. get_object_position(object_name="red_cube")
   # Response: [0.3, 0.3, 0.05]

2. get_object_position(object_name="blue_cube")
   # Response: [0.4, 0.4, 0.05]

3. pick_up_object(object_name="red_cube")
   # Response: {"success": true, "message": "Picked up red_cube"}

4. place_on_object(target_object="blue_cube")
   # Response: {"success": true, "message": "Placed object on blue_cube"}
```

**Conversation Output:**
```
Assistant: I'll stack the red cube on the blue cube. Let me first check their positions.

[Tools: get_object_position × 2]

The red cube is at [0.30, 0.30, 0.05] and the blue cube is at [0.40, 0.40, 0.05].
Now I'll pick up the red cube.

[Tool: pick_up_object]

I've successfully picked up the red cube. Now I'll place it on top of the blue cube.

[Tool: place_on_object]

Done! The red cube is now stacked on top of the blue cube.
```

**Why This Approach:**
- Natural conversation flow with explanations
- Progressive tool execution with user updates
- Error handling at each step
- Visual confirmation available via get_panorama

#### Error Handling in Interactive Mode

**Tool Execution Errors:**

All tool calls return structured results:
```python
{
    "success": true|false,
    "error": "error message if failed",
    "message": "human-readable description",
    "data": {...}  # tool-specific data
}
```

**LLM Response to Errors:**

```
User: "Pick up the yellow cube"

Tool: get_object_position(object_name="yellow_cube")
Error: "Object 'yellow_cube' not found in scene"
Assistant: I'm sorry, but there's no yellow cube in the current scene. Let me check what objects are available.

[Tool: get_all_objects]

The scene contains: red_cube, blue_cube, and green_cube. Would you like me to pick up one of these instead?
```

**Graceful Degradation:**
- LLM explains the error in user-friendly terms
- Offers alternatives based on available information
- Uses query tools to provide context
- Maintains conversational flow despite failures

### Prompt Engineering

Both modes use carefully crafted prompts to ensure reliable LLM behavior.

#### Key Prompting Strategies

**1. Structured Output Formats**

All batch mode prompts enforce JSON output:
```
"Your response must contain ONLY the JSON object and nothing else."
```

This ensures parseable responses for automated execution.

**2. Chain-of-Thought Reasoning**

Every plan includes a `reasoning` field:
```json
{
  "reasoning": "Step-by-step explanation of the approach..."
}
```

This improves plan quality by forcing the LLM to articulate its logic.

**3. Explicit Constraints**

Prompts repeat critical constraints multiple times:
- "IMPORTANT: Avoid gripper collision at all costs"
- "CRITICAL: Check that the placement position is not occupied"
- "The gripper can ONLY grab objects from above, not from sides"

Repetition reinforces safety-critical behaviors.

**4. Visual + Textual Context**

All prompts include both:
- **Panorama image**: Visual understanding of 3D layout
- **Object positions**: Precise numerical coordinates
- **Object dimensions**: Size information for collision planning

This multimodal approach combines spatial intuition with numerical precision.

**5. Example-Driven Learning**

Prompts include concrete examples:
- Simple pick-and-place
- Ground placement with specific coordinates
- Gripper rotation for collision avoidance

Examples establish patterns for the LLM to follow.

**6. Role-Based Prompting**

Each phase has a distinct role:
- **Planner**: "You are a robotic task planner"
- **Reviewer**: "You are a critical reviewer evaluating plan feasibility"
- **Refiner**: "You are a plan improver incorporating feedback"

Role differentiation improves objectivity and focus.

#### Prompt Refinement Tips

**When modifying prompts:**

1. **Test incrementally**: Change one aspect at a time
2. **Use specific examples**: Generic descriptions are ambiguous
3. **Repeat critical rules**: Safety constraints need emphasis
4. **Provide visual context**: Panoramas improve spatial reasoning
5. **Enforce structure**: Strict output formats enable automation
6. **Include error cases**: Show how to handle common failures

**Common pitfalls to avoid:**
- Vague spatial descriptions ("nearby", "close to")
- Contradictory instructions
- Missing output format specifications
- Insufficient examples
- Overlong prompts (focus on essentials)

### Token Optimization

The system uses several strategies to minimize API costs while maintaining quality.

#### Prompt Caching

**What is cached:**

Anthropic's prompt caching stores frequently-used content:

1. **Panorama images** (batch mode):
   - Cached after first use in planning phase
   - Reused in all review and refinement calls
   - Typical savings: 1000-2000 tokens per cached image

2. **Review system prompt** (batch mode):
   - Static content cached across all review calls
   - Typical savings: 500-800 tokens per cached prompt

3. **Conversation history** (interactive mode):
   - Recent messages cached for context
   - Reduces cost of long conversations

**Cache configuration:**
```python
{
    "type": "image",
    "source": {"type": "base64", "data": image_data},
    "cache_control": {"type": "ephemeral"}  # Cache this content
}
```

**Cost impact:**
- Uncached: ~3000 tokens per validation cycle
- Cached: ~500-800 tokens per validation cycle
- **Savings: 70-75% reduction in token usage**

#### Model Selection

**Development recommendations:**

- **Haiku** (`claude-3-5-haiku-20241022`):
  - Fastest, most cost-efficient
  - Recommended for development and testing
  - Sufficient for most manipulation tasks
  - Cost: ~$0.25 per million input tokens

- **Sonnet** (`claude-3-5-sonnet-20241022`):
  - Better spatial reasoning
  - Use for complex tasks or dense scenes
  - Cost: ~$3 per million input tokens

- **Opus** (`claude-opus-4-20250514`):
  - Highest capability
  - Only needed for extremely complex scenarios
  - Cost: ~$15 per million input tokens

**When to upgrade:**
- Plans consistently fail validation → Try Sonnet
- Complex stacking with >5 objects → Try Sonnet/Opus
- Development/testing → Stick with Haiku

#### Monitoring Usage

**Batch mode logging:**

All LLM calls are logged with token counts:
```
[LLM Request] Model: claude-3-5-haiku-20241022
[LLM Response] Tokens: 1234 in, 567 out
Cache: 1000 tokens read, 0 tokens written
```

**Interactive mode tracking:**

Streamlit sidebar shows real-time token usage:
- Input tokens (cumulative)
- Output tokens (cumulative)
- Per-message breakdown

**Typical costs:**
- Batch mode simple task: $0.01-0.05
- Batch mode complex task: $0.10-0.25
- Interactive session (10 messages): $0.05-0.15

## Architecture

### Component Hierarchy

```
main.py / main_interactive.py / main_visual.py (Entry Points)
├── SimulationState (Physics time tracking)
├── ObjectManager (Object registry and queries)
├── CameraManager (Panorama capture)
├── RobotController (Robot operations)
│   ├── IK movement primitives
│   ├── Gripper control
│   └── High-level pick/place operations
├── LLMValidator (Batch mode only)
│   ├── Planning phase
│   ├── Review phase
│   └── Refinement phase
├── LLMController (Batch mode only)
│   └── Plan execution
├── InteractiveLLMController (Interactive mode only)
│   ├── Tool definitions
│   ├── Tool execution
│   └── Conversation management
└── SimulationLogger (Comprehensive logging)
```

**Execution Mode Notes:**
- **main.py**: Batch mode - Single process, direct GUI access
- **main_interactive.py**: Interactive mode client - Connects to visual server or runs headless
- **main_visual.py**: GUI server for macOS - Enables visualization for interactive mode

### Data Flow

**Batch Mode:**
```
Scene YAML → ObjectManager → Camera → Panorama
                                         ↓
Task Description + Panorama → LLMValidator → Validated Plan
                                                    ↓
Validated Plan → LLMController → RobotController → Physical Execution
```

**Interactive Mode:**
```
[main_visual.py] PyBullet GUI Server (macOS only)
        ↑ p.SHARED_MEMORY connection
        ↓
[main_interactive.py] Streamlit Interface
        ↓
User Message → InteractiveLLMController → Claude API (with tools)
                                              ↓
Tool Calls → Tool Execution → RobotController/ObjectManager/CameraManager
                                              ↓
Tool Results → Claude API → User Response → Streamlit UI
```

**macOS Dual-Process Architecture:**
- Process 1 (`main_visual.py`): Owns main thread, creates GUI window, runs as `p.GUI_SERVER`
- Process 2 (`main_interactive.py`): Runs Streamlit, connects via `p.SHARED_MEMORY`
- Communication: Shared memory (no network overhead)
- Both processes share the same PyBullet simulation state

### Key Design Patterns

**1. Manager Pattern**

Each subsystem has a dedicated manager:
- `ObjectManager`: Object lifecycle and queries
- `CameraManager`: Image capture and storage
- `RobotController`: Robot state and operations

**Benefits:**
- Clear separation of concerns
- Easy to test individual components
- Reusable across both modes

**2. Configuration-First Design**

All constants in [src/config.py](src/config.py):
- Physics parameters
- Robot thresholds
- Camera settings
- LLM configuration

**Benefits:**
- Single source of truth
- No magic numbers in code
- Easy parameter tuning

**3. Stateless LLM Controllers**

LLM controllers don't maintain conversation state:
- State passed as parameters
- No hidden dependencies
- Easier to reason about behavior

**4. Structured Logging**

Dedicated loggers for different concerns:
- `app_logger`: Application events
- `llm_logger`: LLM interactions (batch)
- `robot_logger`: Robot operations
- `interactive_logger`: Chat and tools (interactive)

**Benefits:**
- Easy to trace issues
- Separate log streams for analysis
- Token usage tracking

## Configuration

### Key Configuration Files

**1. `.env` - API Configuration**
```bash
ANTHROPIC_API_KEY=your_key_here
ANTHROPIC_MODEL=claude-3-5-haiku-20241022
```

**2. `src/config.py` - System Parameters**

Important settings:
```python
# LLM Configuration
MAX_VALIDATION_ITERATIONS = 3  # Batch mode validation cycles
VALIDATION_MODEL = None  # Override model for validation

# Robot Control
THRESHOLD_OVER_TARGET = 0.02  # Movement precision (meters)
THRESHOLD_PRECISE = 0.005  # Precise positioning threshold
LINEAR_MOVEMENT_SPEED = 0.01  # Smooth movement velocity (m/s)

# Pick/Place Offsets
OVER_TARGET_Z = 0.5  # High overhead position
PICK_CLOSE_OFFSET = 0.15  # Approach height
PLACE_ON_TARGET_OFFSET = 0.12  # Stacking height

# Camera Settings
CAMERA_IMAGE_WIDTH = 640
CAMERA_IMAGE_HEIGHT = 480
PANORAMA_FORMAT = 'JPEG'

# Logging
LOGS_FOLDER = 'logs'
LOG_SESSION_NAME = None  # Auto-timestamp if None
```

**3. `scenes/*.yaml` - Scene Definitions**

See [Scene Configuration](#scene-configuration) section above.

### Tuning Parameters

**For better precision:**
- Decrease `THRESHOLD_PRECISE` (warning: slower execution)
- Increase `IK_MAX_ITERATIONS`
- Use `move_to_target_smooth()` more often

**For faster execution:**
- Increase `THRESHOLD_OVER_TARGET`
- Decrease `STABILIZATION_LOOP_STEPS`
- Use `move_to_target()` instead of smooth movement

**For better LLM performance:**
- Increase `MAX_VALIDATION_ITERATIONS`
- Use better model (Haiku → Sonnet)
- Modify prompts for more detailed reasoning

**For cost reduction:**
- Use Haiku model
- Reduce panorama resolution
- Decrease `MAX_VALIDATION_ITERATIONS`

## Troubleshooting

### Common Issues

**1. "ANTHROPIC_API_KEY not found in .env file"**

**Solution:**
- Ensure `.env` file exists in project root
- Check file contains `ANTHROPIC_API_KEY=your_key_here`
- Verify no extra spaces or quotes around the key
- Try absolute path: `python -c "from dotenv import load_dotenv; load_dotenv(); import os; print(os.getenv('ANTHROPIC_API_KEY'))"`

**2. "Object 'object_name' not found"**

**Solution:**
- Check object exists in scene YAML file
- Verify object `name` field matches exactly (case-sensitive)
- Ensure object loaded successfully (check logs)
- Run `python main.py --scene your_scene` and check console output

**3. "No panorama images found matching pattern"**

**Solution:**
- Ensure `images/` directory exists
- Check panorama captured after stabilization
- Look for errors during camera initialization in logs
- Verify `IMAGES_FOLDER` setting in config.py

**4. "Plan validation failed after max iterations"**

**Causes:**
- Task is impossible given scene configuration
- Objects too close together (collision unavoidable)
- Objects outside robot workspace
- Ambiguous task description

**Solutions:**
- Simplify the task
- Adjust object positions in scene YAML (more spacing)
- Make task description more explicit
- Increase `MAX_VALIDATION_ITERATIONS` in config
- Try better model (Haiku → Sonnet)

**5. "StreamlitAPIException: Unable to convert content to image"**

**Solution:**
- Check panorama format matches expectation (JPEG vs PNG)
- Verify image data is valid base64
- Ensure PIL/Pillow is correctly installed
- Check `PANORAMA_FORMAT` in config.py

**6. "Robot gets stuck or moves erratically"**

**Causes:**
- IK solver unable to find solution
- Target position outside workspace
- Joint limits reached

**Solutions:**
- Check target coordinates are within workspace bounds
- Increase `IK_RESIDUAL_THRESHOLD` for more lenient solutions
- Adjust `ARM_MOTOR_FORCE` for stronger motors
- Add more stabilization steps

**7. "Interactive mode: Conversation becomes slow"**

**Causes:**
- Long conversation history
- Many panorama captures (large images in context)

**Solutions:**
- Restart the Streamlit app periodically
- Avoid requesting panoramas unnecessarily
- Use `streamlit clear_cache` to reset
- Check token usage in sidebar (context window limits)

**8. "macOS: NSInternalInconsistencyException - NSWindow should only be instantiated on the main thread"**

**Cause:**
- Attempting to run interactive mode directly on macOS without visual server
- macOS enforces GUI window creation on main thread only
- Streamlit runs code in background thread

**Solution:**
- Use the two-terminal setup for interactive mode on macOS:
  - Terminal 1: `python main_visual.py`
  - Terminal 2: `streamlit run main_interactive.py`
- Or use headless mode (no GUI, panorama captures only)
- Batch mode is unaffected and works normally

**9. "Interactive mode connects but robot doesn't move in GUI"**

**Causes:**
- Visual server and Streamlit client loaded different scenes
- Scene names don't match between processes

**Solutions:**
- Ensure both processes use the same `--scene` argument
- Restart both Terminal 1 (visual server) and Terminal 2 (Streamlit)
- Check console output for scene loading confirmation
- Verify object names match in both logs

### Debug Mode

**Enable verbose logging:**

1. Edit `src/logger.py` and set log level to DEBUG
2. Check `logs/` directory for detailed execution logs
3. Review LLM request/response logs for prompt issues

**Visualize robot trajectories:**

Debug trails are automatically drawn when enabled:
- Blue line: Target trajectory
- Red line: Actual end-effector path

Control with `TRAIL_DURATION` in config.py.

**Capture intermediate panoramas:**

Panoramas are automatically saved after each operation:
```
images/panorama_000_initial_stabilized.jpg
images/panorama_001_pick_up_red_cube.jpg
images/panorama_002_place_on_blue_cube.jpg
```

Review these to see what the LLM sees during planning.

### Getting Help

**Check logs first:**
- Application log: `logs/session_YYYYMMDD_HHMMSS.log`
- Console output for real-time errors
- LLM request/response logs for prompt debugging

**Review documentation:**
- [CLAUDE.md](CLAUDE.md) - Developer guide with architectural details
- [scenes/README.md](scenes/README.md) - Scene configuration format
- Prompt files in `prompts/` - LLM behavior configuration

**Common log patterns:**
- `[ERROR]`: Critical failure, execution stopped
- `[WARNING]`: Non-critical issue, execution continued
- `[LLM Request]`: Outgoing API call to Claude
- `[LLM Response]`: API response with token counts
- `[Tool Call]`: Interactive mode tool execution

---

## License

[Specify your license here]

## Contributors

[List contributors here]

## Acknowledgments

- Built with [PyBullet](https://pybullet.org/) physics simulation
- Powered by [Anthropic's Claude](https://www.anthropic.com/claude) AI
- Franka Panda robot model from PyBullet examples
