# Claude AI Robotic Arm Control System

A sophisticated PyBullet-based robotics simulation featuring a **Franka Panda robot arm** controlled by **Claude AI (Anthropic)** through natural language commands. The system supports both autonomous batch task execution with multi-stage LLM validation and interactive conversational control modes.

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Three Execution Modes](#three-execution-modes)
  - [1. Batch Mode (main.py)](#1-batch-mode-mainpy)
  - [2. Console Interactive Mode (main_console.py)](#2-console-interactive-mode-main_consolepy)
  - [3. Streamlit Interactive Mode (main_interactive.py)](#3-streamlit-interactive-mode-main_interactivepy)
- [Scene System](#scene-system)
- [How the Models Work Together](#how-the-models-work-together)
  - [Batch Mode: Multi-Stage Validation Workflow](#batch-mode-multi-stage-validation-workflow)
  - [Interactive Mode: Tool-Based Conversational Control](#interactive-mode-tool-based-conversational-control)
- [Prompt Caching Strategy](#prompt-caching-strategy)
- [Console Commands](#console-commands)
- [Architecture Overview](#architecture-overview)
- [Project Structure](#project-structure)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Overview

This project demonstrates advanced AI-powered robotics control by integrating:

- **PyBullet** physics simulation with realistic Franka Panda robot dynamics
- **Claude AI (Anthropic)** for natural language understanding and task planning
- **Multi-stage validation** system for autonomous task execution
- **Interactive conversational control** for real-time human-in-the-loop operation
- **Vision-based scene analysis** through panoramic camera captures
- **Prompt caching** for cost-efficient API usage

The system bridges the gap between high-level natural language instructions and low-level robot control, showcasing practical AI applications in robotics.

---

## Key Features

- **Natural Language Control**: Command robot using plain English (e.g., "Stack the red cube on the blue cube")
- **Vision Analysis**: Multi-view panoramic camera system for scene understanding
- **Plan Validation**: Autonomous critique-refinement loop ensures plan reliability before execution
- **Spatial Reasoning**: Combines visual analysis with precise object position data
- **Interactive Modes**: Choose between autonomous batch execution or conversational control
- **Prompt Caching**: Reduces API costs by up to 90% through intelligent caching
- **Comprehensive Logging**: Detailed logs of robot operations, LLM interactions, and token usage
- **Modular Scene System**: YAML-based scene configuration for easy experimentation

---

## Prerequisites

### Required Software

- **Python**: 3.8 or higher
- **Operating System**: Linux, macOS, or Windows with WSL2
- **PyBullet**: Physics simulation engine
- **Anthropic API Key**: For Claude AI access

### Hardware Requirements

- **CPU**: Modern multi-core processor
- **RAM**: 4GB minimum (8GB recommended)
- **GPU**: Not required (PyBullet runs on CPU)
- **Display**: Required for GUI visualization

---

## Installation

### 1. Clone the Repository

```bash
git clone <repository-url>
cd milab_bead_j0p7mf_2
```

### 2. Create Virtual Environment (Recommended)

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

**Requirements breakdown:**

```
# Core dependencies
pybullet>=3.2.5          # Physics simulation engine
anthropic>=0.25.0        # Claude AI API client
numpy>=1.24.0            # Numerical computations
pillow>=10.0.0           # Image processing for panoramas
pyyaml>=6.0              # Scene configuration parsing
python-dotenv>=1.0.0     # Environment variable management

# Interactive mode dependencies
streamlit>=1.28.0        # Web-based chat interface
rich>=13.0.0             # Terminal formatting for console mode
```

### 4. Configure Environment Variables

Create a `.env` file in the project root:

```bash
# .env file
ANTHROPIC_API_KEY=your_anthropic_api_key_here
ANTHROPIC_MODEL=claude-sonnet-4-5-20250929
```

**How to get your API key:**

1. Sign up at [Anthropic Console](https://console.anthropic.com/)
2. Navigate to API Keys section
3. Create a new API key
4. Copy the key to your `.env` file

---

## Configuration

All system parameters are centralized in [src/config.py](src/config.py). Key configuration options:

### Robot Parameters

```python
END_EFFECTOR_INDEX = 11          # Gripper link index
LINEAR_MOVEMENT_SPEED = 0.1      # Movement speed (m/s)
THRESHOLD_PRECISE = 0.001        # Position accuracy (meters)
```

### Physics Settings

```python
GRAVITY = -9.81                  # Gravity (m/s²)
TIME_STEP = 0.01                 # Simulation step (seconds)
STABILIZATION_LOOP_STEPS = 2500  # Physics stabilization cycles
```

### LLM Settings

```python
MAX_VALIDATION_ITERATIONS = 3    # Critique-refinement cycles
INTERACTIVE_MAX_TOKENS = 4096    # Max tokens per response
ENABLE_PROMPT_CACHING = True     # Use Anthropic prompt caching
PANORAMA_QUALITY = 75            # JPEG quality (affects tokens)
```

### Camera Configuration

```python
CAMERA_IMAGE_WIDTH = 640         # Camera resolution
CAMERA_IMAGE_HEIGHT = 480
CAMERA_FOV = 60                  # Field of view (degrees)
CAMERA_YAW_ANGLES = [0, 90, 180, 270, 0]  # 5-view panorama
```

---

## Three Execution Modes

### 1. Batch Mode (main.py)

**Purpose**: Autonomous task execution with rigorous LLM validation

**Usage:**

```bash
python main.py --scene default
python main.py --scene scene_01
```

**Workflow:**

1. Loads scene from YAML configuration
2. Initializes PyBullet simulation and robot
3. Stabilizes physics for 2500 steps
4. Captures 5-view panoramic scene image
5. Runs **multi-stage LLM validation workflow** (see below)
6. Executes validated plan using robot controller
7. Logs results and exits

**When to use:**
- Autonomous operation without human intervention
- Batch processing of multiple tasks
- Benchmarking and evaluation
- Scenarios requiring validated plans before execution

**API Usage:**
- **Model**: Claude Sonnet 4.5 (configurable)
- **Stages**: 5 separate LLM calls (vision, spatial, planning, review, refinement)
- **Cost Optimization**: Caches panorama image and system prompts

---

### 2. Console Interactive Mode (main_console.py)

**Purpose**: Terminal-based conversational robot control with continuous physics

**Usage:**

```bash
python main_console.py --scene default
```

**Features:**

- **Text-based chat interface** using Rich formatting
- **Continuous physics simulation** during user input (non-blocking)
- **Real-time object and gripper state display**
- **Tool execution visualization** (verbose mode)
- **Special commands** for status, help, and control

**Workflow:**

1. Initializes simulation with GUI visualization
2. Enters infinite chat loop
3. User types commands while physics runs continuously
4. Claude processes messages using tool calling
5. Robot executes actions in real-time
6. Results displayed in formatted terminal output

**When to use:**
- Development and debugging
- Quick experimentation without web browser
- Remote SSH sessions (GUI visible locally)
- Performance testing (lower overhead than Streamlit)

**API Usage:**
- **Model**: Claude Sonnet 4.5 (configurable)
- **Tool Calling**: Native Anthropic tool use API
- **Cost Optimization**: Caches system prompt and tool definitions

---

### 3. Streamlit Interactive Mode (main_interactive.py)

**Purpose**: Web-based conversational robot control with rich UI

**Usage:**

```bash
streamlit run main_interactive.py
```

Access the web interface at `http://localhost:8501`

**Features:**

- **Modern chat interface** with markdown rendering
- **Sidebar with scene information** (objects, gripper state, token usage)
- **Inline panorama display** within chat
- **Tool execution details** in expandable sections
- **Persistent conversation history** across page refreshes

**Workflow:**

1. Streamlit caches simulation initialization
2. User interacts through web chat interface
3. Claude processes messages using tool calling
4. Results displayed with rich formatting
5. Session state maintained automatically

**When to use:**
- User-friendly interface for demonstrations
- Non-technical users controlling robot
- Multiple simultaneous users (each gets own session)
- Scenarios requiring visual feedback

**API Usage:**
- **Model**: Claude Sonnet 4.5 (configurable)
- **Tool Calling**: Native Anthropic tool use API
- **Cost Optimization**: Caches system prompt and tool definitions

---

## Scene System

Scenes are defined in YAML files located in `scenes/` directory.

### Scene Structure

```yaml
metadata:
  name: "Scene Name"
  description: "Scene description for logging"

objects:
  - name: "red_cube"
    type: "cube"
    position: [0.3, 0.3, 0.05]  # [x, y, z] in meters
    color: [1, 0, 0, 1]         # RGBA (0-1 range)
    scale: 1.0                  # Size multiplier

  - name: "blue_cube"
    type: "cube"
    position: [0.4, 0.4, 0.05]
    color: [0, 0, 1, 1]
    scale: 1.0

task:
  description: "Stack the red cube on top of the blue cube"
```

### Robot Workspace Limits

The robot can reach positions within:

- **X-axis**: -0.5 to 0.7 meters
- **Y-axis**: -0.7 to 0.7 meters
- **Z-axis**: 0 to 0.8 meters

Objects outside this workspace cannot be manipulated.

### Available Scenes

```bash
# List available scenes
ls scenes/*.yaml

# Load specific scene
python main.py --scene scene_01
python main_console.py --scene scene_02
```

### Creating Custom Scenes

1. Copy `scenes/default.yaml` to `scenes/my_scene.yaml`
2. Modify object positions, colors, and task description
3. Run with `python main.py --scene my_scene`

---

## How the Models Work Together

### Batch Mode: Multi-Stage Validation Workflow

The batch mode (`main.py`) implements a **5-stage LLM workflow** with iterative critique-refinement:

```
[Scene Image] → Vision Analysis → Spatial Analysis → Planning
                                                        ↓
                                              ← Refinement ← Review
                                              (Loop up to 3x)
                                                        ↓
                                                  [Execute Plan]
```

#### Stage 1: Vision Analysis

**Purpose**: Understand what the robot can "see" from visual data

**Process:**

1. Loads panoramic scene image (5 views stitched together)
2. Sends to Claude with vision analysis prompt
3. Claude identifies:
   - Object visibility and positions
   - Spatial relationships (near, far, blocking)
   - Reachability assessment

**Prompt Used**: `prompts/vision_analysis_system_prompt.txt` + `prompts/vision_analysis_user_prompt.txt`

**Why this stage exists:**
- Visual understanding provides context that pure coordinates miss
- Identifies occlusions and blocking relationships
- Validates that objects are actually visible in the scene

**Example Output:**

```json
{
  "visible_objects": [
    {
      "name": "red_cube",
      "approximate_position": "center-left of workspace",
      "reachable": true,
      "blocking_objects": []
    },
    {
      "name": "blue_cube",
      "approximate_position": "center-right of workspace",
      "reachable": true,
      "blocking_objects": []
    }
  ],
  "workspace_clear": true
}
```

---

#### Stage 2: Spatial Analysis

**Purpose**: Combine visual understanding with precise position data

**Process:**

1. Receives vision analysis output from Stage 1
2. Gets exact object positions from ObjectManager
3. Claude combines visual + geometric data to understand:
   - Exact blocking relationships
   - Whether objects are stable
   - Spatial constraints for manipulation

**Prompt Used**: `prompts/spatial_analysis_system_prompt.txt` + `prompts/spatial_analysis_user_prompt.txt`

**Why this stage exists:**
- Vision alone lacks precision for motion planning
- Coordinates alone lack context about blocking
- Combines strengths of both modalities

**Example Output:**

```json
{
  "spatial_relationships": [
    {
      "object": "green_cube",
      "blocks": ["red_cube"],
      "reason": "green_cube at [0.35, 0.35] is between robot and red_cube at [0.3, 0.3]"
    }
  ],
  "manipulation_order": ["green_cube", "blue_cube", "red_cube"],
  "constraints": [
    "Must move green_cube first to access red_cube"
  ]
}
```

---

#### Stage 3: Initial Planning

**Purpose**: Generate executable command sequence from analyses

**Process:**

1. Receives vision analysis + spatial analysis
2. Receives task description from scene YAML
3. Claude generates plan with commands:
   - `pick_up(object_name)`
   - `place(x, y, z)`
   - `place_on(object_name)`
   - `rotate_gripper_90()`
   - `reset_gripper_orientation()`

**Prompt Used**: `prompts/planning_system_prompt.txt` + `prompts/planning_user_prompt.txt`

**Why this stage exists:**
- Translates high-level understanding into executable actions
- Uses analyses to avoid infeasible plans
- Generates structured output for execution

**Example Output:**

```json
{
  "plan": "Stack red cube on blue cube",
  "commands": [
    {"action": "pick_up", "object": "red_cube"},
    {"action": "place_on", "object": "blue_cube"}
  ],
  "reasoning": "Both cubes are reachable with no blocking objects"
}
```

---

#### Stage 4: Review (Critique)

**Purpose**: Validate plan against scene constraints and task requirements

**Process:**

1. Receives proposed plan + panorama image + task description
2. Critic LLM (separate Claude call) evaluates:
   - Correctness (does it achieve the task?)
   - Feasibility (are commands executable?)
   - Safety (will it cause collisions?)
   - Efficiency (is it optimal?)
3. Returns approval OR detailed critique with suggestions

**Prompt Used**: `prompts/review_system_prompt.txt` + `prompts/review_user_prompt.txt`

**Why this stage exists:**
- Self-validation catches planning errors before execution
- Prevents robot from attempting impossible tasks
- Iterative improvement through refinement loop

**Example Output (Rejection):**

```json
{
  "approved": false,
  "critique": "Plan attempts to pick up green_cube which blocks red_cube, but then tries to place red_cube without moving green_cube first",
  "suggestions": [
    "Move green_cube out of the way before accessing red_cube",
    "Consider placement positions that don't block other objects"
  ]
}
```

**Example Output (Approval):**

```json
{
  "approved": true,
  "critique": "Plan correctly identifies manipulation order and uses appropriate commands"
}
```

---

#### Stage 5: Refinement

**Purpose**: Fix plan based on critique feedback

**Process:**

1. Triggered only if Stage 4 rejected the plan
2. Receives:
   - Original plan
   - Critique with suggestions
   - Vision + spatial analyses
   - Task description
3. Claude generates improved plan addressing critique
4. Returns to Stage 4 for re-review

**Prompt Used**: `prompts/refinement_system_prompt.txt` + `prompts/refinement_user_prompt.txt`

**Why this stage exists:**
- Enables iterative improvement without human intervention
- Learns from mistakes in previous iterations
- Converges toward valid solution

**Loop Limit**: Maximum 3 critique-refinement cycles (`MAX_VALIDATION_ITERATIONS = 3`)

**Example Output:**

```json
{
  "plan": "Move blocking object, then stack red on blue",
  "commands": [
    {"action": "pick_up", "object": "green_cube"},
    {"action": "place", "position": [0.5, 0.5, 0.05]},
    {"action": "pick_up", "object": "red_cube"},
    {"action": "place_on", "object": "blue_cube"}
  ],
  "reasoning": "Addressed critique by first moving green_cube to clear path to red_cube"
}
```

---

#### Execution

Once a plan is validated (or max iterations reached):

1. `LLMController.execute_plan()` processes command list
2. Each command maps to `RobotController` method:
   - `pick_up()` → IK motion + gripper close
   - `place()` → IK motion + gripper open
   - `place_on()` → Calculate target position + place
   - `rotate_gripper_90()` → Orientation adjustment
   - `reset_gripper_orientation()` → Return to default
3. Physics simulation runs during execution
4. Logs and panoramas captured at key points

---

### Interactive Mode: Tool-Based Conversational Control

Interactive modes (`main_console.py` and `main_interactive.py`) use a fundamentally different architecture:

```
User Message → Claude (with tools) → Tool Calls → Robot Actions
                  ↓                       ↓
            Assistant Response ← Tool Results
```

#### Tool Definitions

Claude has access to **10 tools** for scene understanding and robot control:

**Information Tools:**

1. **`get_gripper_position()`** - Query current end-effector position
2. **`get_all_objects()`** - List all objects with positions and dimensions
3. **`get_object_position(object_name)`** - Get specific object position
4. **`get_panorama()`** - Capture and return panorama image (base64)

**Action Tools:**

5. **`pick_up(object_name)`** - Pick up specified object
6. **`place(x, y, z)`** - Place held object at coordinates
7. **`place_on(target_object)`** - Place held object on another object
8. **`open_gripper()`** - Open gripper fingers
9. **`close_gripper()`** - Close gripper fingers
10. **`move_gripper(x, y, z)`** - Move gripper to position

#### Tool Calling Flow

**Implementation**: [src/interactive_llm_controller.py](src/interactive_llm_controller.py)

```python
def handle_message(user_message, conversation_history):
    # 1. Build messages with conversation history
    messages = conversation_history + [{"role": "user", "content": user_message}]

    # 2. Call Claude with tools available
    response = client.messages.create(
        model=self.model,
        max_tokens=self.max_tokens,
        system=[
            {
                "type": "text",
                "text": self.system_prompt,
                "cache_control": {"type": "ephemeral"}  # Cache system prompt
            }
        ],
        tools=self.get_tool_definitions(),  # Tool definitions
        messages=messages
    )

    # 3. Process tool calls in response
    while response.stop_reason == "tool_use":
        tool_results = []
        for content_block in response.content:
            if content_block.type == "tool_use":
                # Execute tool
                result = self._execute_tool(content_block.name, content_block.input)
                tool_results.append(result)

        # 4. Send tool results back to Claude
        messages.append({
            "role": "assistant",
            "content": response.content
        })
        messages.append({
            "role": "user",
            "content": tool_results
        })

        # 5. Continue conversation with tool results
        response = client.messages.create(...)

    # 6. Extract final text response
    return extract_text_response(response)
```

#### Why Tool-Based Architecture?

**Advantages over batch validation:**

- **Conversational**: Natural back-and-forth interaction
- **Flexible**: User can change plans mid-execution
- **Real-time feedback**: Immediate response to user queries
- **Exploration**: User can query scene state at any time
- **Error recovery**: User can correct mistakes interactively

**Trade-offs:**

- No multi-stage validation (user provides validation)
- Requires human in the loop
- Less suitable for autonomous operation

#### Tool Execution Examples

**Example 1: Simple pickup**

```
User: "Pick up the red cube"

Claude thinks:
  1. Should query objects first to verify red_cube exists
  2. Then use pick_up tool

Tool calls:
  - get_all_objects() → Returns list with red_cube at [0.3, 0.3, 0.05]
  - pick_up("red_cube") → Executes pickup sequence

Response: "I've picked up the red cube from position [0.3, 0.3, 0.05]."
```

**Example 2: Complex stacking**

```
User: "Stack all cubes in a tower"

Claude thinks:
  1. Need to know what cubes exist
  2. Plan stacking order (bottom to top)
  3. Execute picks and places

Tool calls:
  - get_all_objects() → Returns red, green, blue cubes
  - pick_up("red_cube")
  - place(0.4, 0.4, 0.05)  # Center position
  - pick_up("green_cube")
  - place_on("red_cube")
  - pick_up("blue_cube")
  - place_on("green_cube")

Response: "I've stacked all three cubes into a tower at the center of the workspace."
```

**Example 3: Scene exploration**

```
User: "Show me what you see"

Claude thinks:
  1. Should capture panorama for visual context
  2. List objects with positions

Tool calls:
  - get_panorama() → Returns base64 image
  - get_all_objects() → Returns object list

Response: "Here's a panorama of the scene. I can see 3 cubes:
- red_cube at [0.3, 0.3, 0.05]
- green_cube at [0.35, 0.35, 0.05]
- blue_cube at [0.4, 0.4, 0.05]
All are within reach and ready for manipulation."

[Panorama image displayed]
```

---

## Prompt Caching Strategy

**Anthropic Prompt Caching** reduces API costs by caching frequently reused content.

### What is Prompt Caching?

- **Cache Duration**: 5 minutes (configurable)
- **Cache Keys**: Based on exact content match
- **Cost Reduction**: Up to 90% for cached tokens
- **Pricing**: Cache writes cost 25% more, cache reads cost 10% of regular tokens

### Batch Mode Caching

**Cached Content:**

1. **Panorama Image** (largest token cost)
   - Cached in vision analysis stage
   - Reused in review stage (same image)
   - Saves ~1500-2000 tokens per review iteration

2. **System Prompts** (moderate token cost)
   - Each stage prompt cached separately
   - Reused if running multiple scenes
   - Saves ~500-1000 tokens per stage

**Implementation Example:**

```python
# Vision analysis with cached panorama
system_content = [
    {
        "type": "text",
        "text": vision_analysis_system_prompt
    },
    {
        "type": "image",
        "source": {
            "type": "base64",
            "media_type": "image/jpeg",
            "data": panorama_base64
        },
        "cache_control": {"type": "ephemeral"}  # Mark for caching
    }
]
```

**Cost Analysis:**

Without caching (3 validation iterations):
```
Vision: 2000 input tokens × $3/MTok = $0.006
Review 1: 2000 input tokens × $3/MTok = $0.006
Review 2: 2000 input tokens × $3/MTok = $0.006
Review 3: 2000 input tokens × $3/MTok = $0.006
Total: $0.024
```

With caching (3 validation iterations):
```
Vision: 2000 input tokens × $3.75/MTok = $0.0075 (cache write)
Review 1: 2000 cached tokens × $0.30/MTok = $0.0006 (cache read)
Review 2: 2000 cached tokens × $0.30/MTok = $0.0006 (cache read)
Review 3: 2000 cached tokens × $0.30/MTok = $0.0006 (cache read)
Total: $0.0093 (61% savings)
```

### Interactive Mode Caching

**Cached Content:**

1. **System Prompt** (~500-800 tokens)
   - Cached on first message
   - Reused for entire conversation session

2. **Tool Definitions** (~1000-1500 tokens)
   - Cached with system prompt
   - Reused for every tool call

**Implementation Example:**

```python
system_content = [
    {
        "type": "text",
        "text": self.system_prompt,
        "cache_control": {"type": "ephemeral"}  # Cache system prompt + tools
    }
]
```

**Cost Analysis:**

10-message conversation without caching:
```
Per message: 1500 system tokens × $3/MTok = $0.0045
10 messages: $0.045
```

10-message conversation with caching:
```
First message: 1500 tokens × $3.75/MTok = $0.005625 (cache write)
Messages 2-10: 1500 tokens × $0.30/MTok × 9 = $0.00405 (cache reads)
Total: $0.009675 (78% savings)
```

### Cache Configuration

Edit `src/config.py`:

```python
# Enable/disable caching
ENABLE_PROMPT_CACHING = True

# Cache duration (None = 5 min default, "1h" = 1 hour costs 2x)
PROMPT_CACHE_TTL = None

# Image quality affects cached token count
PANORAMA_QUALITY = 75  # Lower = fewer tokens but worse quality
```

### Monitoring Cache Performance

Check logs for cache metrics:

```
[LLM] Request completed
  Input tokens: 250
  Cache creation tokens: 1842  ← New content cached
  Cache read tokens: 0
  Output tokens: 156
```

Next request:

```
[LLM] Request completed
  Input tokens: 300
  Cache creation tokens: 0
  Cache read tokens: 1842      ← Reused cached content
  Output tokens: 189
```

---

## Console Commands

### Console Mode Special Commands

When running `main_console.py`, these commands control the interface:

| Command | Description |
|---------|-------------|
| `/help` | Show all available commands and examples |
| `/status` | Display scene info (objects, gripper state, token usage) |
| `/quit` or `/exit` | Exit the console |
| `/clear` | Clear conversation history |
| `/verbose` | Toggle verbose mode (show/hide tool execution details) |

**Example Session:**

```bash
$ python main_console.py --scene default

🤖 ROBOT CONTROL CONSOLE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Interact with the Franka Panda robot using natural language commands.
Type /help for available commands, /quit to exit.

┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃   🎬 Simulation Ready      ┃
┃                             ┃
┃ Scene: Example Scene        ┃
┃ Objects: 3                  ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

You: /status

┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃   Objects in Scene                ┃
┠───────────────┬───────────────────┨
┃ Object Name   │ Position [x,y,z]  ┃
┠───────────────┼───────────────────┨
┃ red_cube      │ [0.300,0.300,0.05]┃
┃ green_cube    │ [0.350,0.350,0.05]┃
┃ blue_cube     │ [0.400,0.400,0.05]┃
┗━━━━━━━━━━━━━━━┷━━━━━━━━━━━━━━━━━━━┛

┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃   Gripper State            ┃
┃                             ┃
┃ 🟢 State: Open              ┃
┃ Position: [0.250,0.250,0.5]┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

You: pick up the red cube

🤖 Thinking...

┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃   🤖 Assistant                     ┃
┃                                    ┃
┃ I've successfully picked up the    ┃
┃ red cube from position [0.3, 0.3,  ┃
┃ 0.05].                             ┃
┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛

🔧 Tools: 1. pick_up

💰 Tokens: 1,842 in / 189 out | Total: 2,031

You: /quit

🎯 Session Summary
┏━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━┓
┃ Metric             ┃   Value ┃
┠────────────────────┼─────────┨
┃ Messages exchanged ┃       2 ┃
┃ Total input tokens ┃   2,142 ┃
┃ Total output tokens┃     345 ┃
┃ Total tokens       ┃   2,487 ┃
┗━━━━━━━━━━━━━━━━━━━━┷━━━━━━━━━┛

👋 Goodbye!
```

### Verbose Mode

Toggle with `/verbose` to see detailed tool execution:

**Normal mode:**
```
🔧 Tools: 1. get_all_objects • 2. pick_up
```

**Verbose mode:**
```
┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃   🔧 Tools Used                                   ┃
┠───┬──────────────────┬──────────┬─────────────────┨
┃ # │ Tool             │ Input    │ Result          ┃
┠───┼──────────────────┼──────────┼─────────────────┨
┃ 1 │ get_all_objects  │ {}       │ [{'name': 'red..┃
┃ 2 │ pick_up          │ {'object │ Successfully pi..┃
┗━━━┷━━━━━━━━━━━━━━━━━━┷━━━━━━━━━━┷━━━━━━━━━━━━━━━━━┛
```

---

## Architecture Overview

### Core Components

```
src/
├── config.py                      # Centralized configuration
├── simulation_state.py            # Time tracking
├── object_manager.py              # Object lifecycle (load, query positions)
├── robot_controller.py            # Low-level robot control (IK, gripper)
├── camera_manager.py              # Multi-view panorama capture
├── scene_loader.py                # YAML scene parsing
├── logger.py                      # Logging (console + file + API calls)
├── llm_controller.py              # Batch mode plan execution
├── llm_validator.py               # Multi-stage validation workflow
└── interactive_llm_controller.py  # Interactive mode tool calling
```

### Key Files

**[src/robot_controller.py](src/robot_controller.py)**
- Inverse kinematics (IK) motion planning
- Gripper control (open/close)
- High-level actions: `pick_up()`, `place()`, `place_on()`
- Orientation control: `rotate_orientation_90()`, `reset_orientation()`

**[src/llm_validator.py](src/llm_validator.py)**
- Multi-stage validation orchestrator
- `_analyze_scene_vision()` - Vision analysis (Stage 1)
- `_analyze_spatial_relationships()` - Spatial analysis (Stage 2)
- `_generate_initial_plan()` - Planning (Stage 3)
- `_critique_plan()` - Review (Stage 4)
- `_refine_plan()` - Refinement (Stage 5)
- `get_validated_plan()` - Main workflow method

**[src/llm_controller.py](src/llm_controller.py)**
- Plan execution engine
- `execute_plan()` - Executes validated command sequences

**[src/interactive_llm_controller.py](src/interactive_llm_controller.py)**
- Conversational control
- `handle_message()` - Process user messages with tool calling
- `_define_tools()` - Tool definitions for Claude
- `_execute_tool()` - Tool execution dispatch

**[src/object_manager.py](src/object_manager.py)**
- Object lifecycle management
- `load_cube()` - Load objects into simulation
- `get_object_center_position()` - Query object positions
- `get_object_dimensions()` - Query object sizes

**[src/camera_manager.py](src/camera_manager.py)**
- Multi-view panorama capture
- Captures 5 views: front (0°), right (90°), back (180°), left (270°), top (-89°)
- Stitches views into single panorama image

**[src/logger.py](src/logger.py)**
- Dual logging: console (Rich) + file (rotating logs)
- LLM request/response tracking with token usage
- API call logging with cache metrics

### Prompt Files

```
prompts/
├── vision_analysis_system_prompt.txt    # Stage 1 system prompt
├── vision_analysis_user_prompt.txt      # Stage 1 user prompt
├── spatial_analysis_system_prompt.txt   # Stage 2 system prompt
├── spatial_analysis_user_prompt.txt     # Stage 2 user prompt
├── planning_system_prompt.txt           # Stage 3 system prompt
├── planning_user_prompt.txt             # Stage 3 user prompt
├── review_system_prompt.txt             # Stage 4 system prompt
├── review_user_prompt.txt               # Stage 4 user prompt
├── refinement_system_prompt.txt         # Stage 5 system prompt
├── refinement_user_prompt.txt           # Stage 5 user prompt
└── interactive_system_prompt.txt        # Interactive mode system prompt
```

### Data Flow (Batch Mode)

```
Scene YAML
    ↓
Scene Loader → Object Manager → PyBullet Simulation
                                       ↓
                              Robot Controller
                                       ↓
                              Camera Manager → Panorama
                                                   ↓
                                            LLM Validator
                                            (5 stages)
                                                   ↓
                                            Validated Plan
                                                   ↓
                                            LLM Controller
                                                   ↓
                                            Robot Controller
                                                   ↓
                                            Execution
```

### Data Flow (Interactive Mode)

```
User Input (Console/Streamlit)
    ↓
Interactive LLM Controller
    ↓
Claude API (with tools)
    ↓
Tool Execution:
  - Query tools → Object Manager / Robot Controller
  - Action tools → Robot Controller → PyBullet
  - Camera tools → Camera Manager → Panorama
    ↓
Tool Results
    ↓
Claude API (continue conversation)
    ↓
Assistant Response → User Interface
```

---

## Project Structure

```
milab_bead_j0p7mf_2/
│
├── main.py                          # Batch mode entry point
├── main_console.py                  # Console interactive mode entry point
├── main_interactive.py              # Streamlit interactive mode entry point
│
├── requirements.txt                 # Python dependencies
├── .env                             # Environment variables (API keys)
├── CLAUDE.md                        # Instructions for Claude Code
├── README.md                        # This file
│
├── src/                             # Source code
│   ├── config.py                    # Configuration parameters
│   ├── simulation_state.py          # Simulation state tracking
│   ├── object_manager.py            # Object management
│   ├── robot_controller.py          # Robot control (IK, gripper, actions)
│   ├── camera_manager.py            # Panorama capture
│   ├── scene_loader.py              # YAML scene parsing
│   ├── logger.py                    # Logging system
│   ├── llm_controller.py            # Batch mode plan execution
│   ├── llm_validator.py             # Multi-stage validation workflow
│   └── interactive_llm_controller.py# Interactive mode tool calling
│
├── prompts/                         # LLM prompt templates
│   ├── vision_analysis_system_prompt.txt
│   ├── vision_analysis_user_prompt.txt
│   ├── spatial_analysis_system_prompt.txt
│   ├── spatial_analysis_user_prompt.txt
│   ├── planning_system_prompt.txt
│   ├── planning_user_prompt.txt
│   ├── review_system_prompt.txt
│   ├── review_user_prompt.txt
│   ├── refinement_system_prompt.txt
│   ├── refinement_user_prompt.txt
│   └── interactive_system_prompt.txt
│
├── scenes/                          # Scene configuration files
│   ├── default.yaml
│   ├── scene_01.yaml
│   ├── scene_02.yaml
│   └── ...
│
├── logs/                            # Generated logs (gitignored)
│   ├── app_YYYYMMDD_HHMMSS.log
│   └── api_calls_YYYYMMDD_HHMMSS.log
│
└── images/                          # Generated panoramas (gitignored)
    ├── panorama_initial_stabilized.jpg
    └── ...
```

---

## Development

### Adding New Robot Actions

**Step 1**: Add method to `RobotController`

```python
# src/robot_controller.py
def rotate_object(self, angle_degrees):
    """Rotate held object by specified angle."""
    # Implementation...
```

**Step 2**: For batch mode, update plan execution

```python
# src/llm_controller.py
def execute_plan(self, plan):
    # ...
    elif action == "rotate_object":
        angle = command.get('angle')
        self.robot_controller.rotate_object(angle)
```

**Step 3**: For batch mode, update planning prompts

Edit `prompts/planning_system_prompt.txt` to document new action.

**Step 4**: For interactive mode, add tool definition

```python
# src/interactive_llm_controller.py
def get_tool_definitions(self):
    return [
        # ...
        {
            "name": "rotate_object",
            "description": "Rotate the held object by specified angle",
            "input_schema": {
                "type": "object",
                "properties": {
                    "angle": {
                        "type": "number",
                        "description": "Rotation angle in degrees"
                    }
                },
                "required": ["angle"]
            }
        }
    ]
```

**Step 5**: Add tool execution handler

```python
# src/interactive_llm_controller.py
def _execute_tool(self, tool_name, tool_input):
    # ...
    elif tool_name == "rotate_object":
        angle = tool_input["angle"]
        self.robot_controller.rotate_object(angle)
        return f"Rotated object by {angle} degrees"
```

### Modifying Validation Logic

Edit `src/llm_validator.py`:

```python
# Change max iterations
# In config.py:
MAX_VALIDATION_ITERATIONS = 5  # Allow more refinement cycles

# Customize critique criteria
def _critique_plan(self, plan, panorama_path, task):
    # Modify review prompts or add custom validation logic
    # ...
```

### Adding New Object Types

Currently only cubes are supported. To add spheres/cylinders:

**Step 1**: Add URDF or geometry loader

```python
# src/object_manager.py
def load_sphere(self, name, position, color, radius):
    sphere_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    sphere_visual = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    sphere_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=sphere_collision,
                                   baseVisualShapeIndex=sphere_visual, basePosition=position)
    self.objects[name] = sphere_id
```

**Step 2**: Update scene loader

```python
# src/scene_loader.py
if obj.type == 'cube':
    object_manager.load_cube(...)
elif obj.type == 'sphere':
    object_manager.load_sphere(name, position, color, obj.radius)
```

**Step 3**: Update prompts to describe new object type

Edit planning prompts to mention sphere handling considerations.

### Debugging

**Enable detailed API logging:**

```python
# src/config.py
LOG_API_CALLS = True
```

Check `logs/api_calls_*.log` for full request/response traces.

**Inspect panoramas:**

Generated panoramas are saved to `images/` directory:
```bash
ls -lh images/
# panorama_initial_stabilized.jpg
# panorama_after_execution.jpg
```

**Monitor token usage:**

Console output shows token counts:
```
[LLM] Request completed
  Input tokens: 2,142 (250 new + 1,892 cached)
  Output tokens: 189
```

**Check validation workflow:**

Logs show each stage:
```
[LLM] Vision analysis completed
[LLM] Spatial analysis completed
[LLM] Initial plan generated
[LLM] Review iteration 1: Plan rejected
[LLM] Refinement completed
[LLM] Review iteration 2: Plan approved
```

---

## Troubleshooting

### Common Issues

**Issue**: `ANTHROPIC_API_KEY not found in .env file`

**Solution**: Create `.env` file with your API key:
```bash
echo "ANTHROPIC_API_KEY=your_key_here" > .env
```

---

**Issue**: `ImportError: No module named 'pybullet'`

**Solution**: Install dependencies:
```bash
pip install -r requirements.txt
```

---

**Issue**: Robot cannot reach object

**Solution**: Check object position in scene YAML. Must be within workspace:
- X: [-0.5, 0.7]
- Y: [-0.7, 0.7]
- Z: [0, 0.8]

---

**Issue**: Validation loop fails after 3 iterations

**Solution**: Either:
1. Increase `MAX_VALIDATION_ITERATIONS` in `config.py`
2. Simplify task in scene YAML
3. Adjust object positions to be more feasible
4. Check logs for critique feedback to understand why plan is rejected

---

**Issue**: Streamlit shows "Connection error"

**Solution**:
1. Check PyBullet GUI is not already running
2. Kill existing PyBullet processes: `pkill -f pybullet`
3. Restart Streamlit: `streamlit run main_interactive.py`

---

**Issue**: High API costs

**Solution**:
1. Enable prompt caching: `ENABLE_PROMPT_CACHING = True`
2. Reduce panorama quality: `PANORAMA_QUALITY = 50`
3. Lower max tokens: `INTERACTIVE_MAX_TOKENS = 2048`
4. Monitor logs for token usage and optimize prompts

---

**Issue**: Robot gripper won't close on object

**Solution**:
1. Check object dimensions (too large?)
2. Verify gripper is positioned correctly (check logs)
3. Increase `GRIPPER_MOVEMENT_STEPS` in config
4. Check if object is too heavy (adjust mass in `load_cube()`)

---

## License

[Specify your license here]

---

## Acknowledgments

- **PyBullet**: Physics simulation framework
- **Anthropic Claude**: AI language model for planning and control
- **Franka Panda Robot**: Robot model and specifications

---

## Contact

For questions or issues, please open an issue on the GitHub repository or contact [your contact info].

---

**Happy Robot Controlling!** 🤖
