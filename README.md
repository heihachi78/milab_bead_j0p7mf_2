# 🤖 Claude AI Robotics Simulation

A PyBullet-based robotics simulation featuring a **Franka Panda robot arm** controlled by **Claude AI** through natural language commands. The system bridges natural language instructions with low-level robot control using vision analysis, spatial reasoning, and intelligent planning.

![Python](https://img.shields.io/badge/python-3.8%2B-blue)
![PyBullet](https://img.shields.io/badge/PyBullet-3.2.6-green)
![Anthropic](https://img.shields.io/badge/Claude-4.5-purple)

---
Take a look at it: https://www.youtube.com/watch?v=uw4upkN-eTM
--

## 📋 Table of Contents

- [Quick Start](#-quick-start)
- [Architecture Overview](#-architecture-overview)
- [Three Operation Modes](#-three-operation-modes)
  - [Batch Mode](#1-batch-mode)
  - [Console Interactive Mode](#2-console-interactive-mode)
  - [Streamlit Interactive Mode](#3-streamlit-interactive-mode)
- [Detailed Workflow Explanations](#-detailed-workflow-explanations)
- [RAG Pipeline](#-rag-pipeline)
- [Database Structure](#-database-structure)
- [Scene System](#-scene-system)
- [Configuration](#-configuration)
- [Prompt Caching](#-prompt-caching-strategy)
- [Logging System](#-logging-system)
- [Development Guide](#-development-guide)
- [Troubleshooting](#-troubleshooting)

---

## 🚀 Quick Start

### Prerequisites

- Python 3.8+
- Anthropic API key ([get one here](https://console.anthropic.com/))
- Linux, macOS, or Windows with WSL

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd milab_bead_j0p7mf_2

# Create and activate virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Set up environment variables
echo "ANTHROPIC_API_KEY=your_key_here" > .env
echo "ANTHROPIC_MODEL=claude-sonnet-4-5-20250929" >> .env
```

### Running the Application

```bash
# Batch mode - autonomous task execution
python main.py --scene default

# Console mode - terminal-based interactive control
python main_console.py --scene default

# Streamlit mode - web-based UI (access at http://localhost:8501)
streamlit run main_interactive.py
```

---

## 🏗 Architecture Overview

The system consists of several layered components:

```
┌─────────────────────────────────────────────────────────────────┐
│                     User Interface Layer                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │  main.py     │  │main_console  │  │ main_interactive.py  │  │
│  │  (Batch)     │  │     .py      │  │    (Streamlit)       │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────────┐
│                     Control Layer                               │
│  ┌────────────────────┐           ┌──────────────────────────┐  │
│  │  LLMValidator      │           │ InteractiveLLMController │  │
│  │  LLMController     │           │                          │  │
│  │  (Batch Planning)  │           │  (Tool-based Control)    │  │
│  └────────────────────┘           └──────────────────────────┘  │
│                │                              │                  │
│                └──────────┬───────────────────┘                  │
│                           ▼                                      │
│              ┌──────────────────────────┐                        │
│              │      RAGRetriever        │                        │
│              │  (Context Augmentation)  │                        │
│              └──────────────────────────┘                        │
└─────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────────┐
│                    Execution Layer                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │RobotController│ │ObjectManager │  │   CameraManager      │  │
│  │   (IK, grip) │  │ (positions)  │  │  (5-view panorama)   │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────────┐
│                    Physics Layer                                │
│                     PyBullet Simulation                         │
│              (Franka Panda, Objects, Physics)                   │
└─────────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────────┐
│                    Data Layer                                   │
│  ┌──────────────────────────┐  ┌─────────────────────────────┐  │
│  │       TaskStore          │  │         ChromaDB            │  │
│  │  (SQLite: executions)    │  │   (Vector embeddings)       │  │
│  └──────────────────────────┘  └─────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Core Components

- **[src/config.py](src/config.py)**: Centralized configuration for all parameters
- **[src/robot_controller.py](src/robot_controller.py)**: Low-level robot control (IK solver, gripper, actions)
- **[src/llm_validator.py](src/llm_validator.py)**: Batch mode planning and validation
- **[src/llm_controller.py](src/llm_controller.py)**: Batch mode plan execution
- **[src/interactive_llm_controller.py](src/interactive_llm_controller.py)**: Interactive mode tool-based control
- **[src/object_manager.py](src/object_manager.py)**: Object lifecycle and position queries
- **[src/camera_manager.py](src/camera_manager.py)**: Multi-view panorama capture
- **[src/scene_loader.py](src/scene_loader.py)**: YAML scene parsing
- **[src/logger.py](src/logger.py)**: Dual logging system (console + file)
- **[src/simulation_state.py](src/simulation_state.py)**: Simulation state tracking
- **[src/task_store.py](src/task_store.py)**: Task history database (SQLite + ChromaDB)
- **[src/rag_retriever.py](src/rag_retriever.py)**: RAG context retrieval and formatting

---

## 🎯 Three Operation Modes

The system provides three distinct modes of operation, each with different architectures and workflows:

### 1. Batch Mode

**Purpose**: Autonomous task execution with validated planning

**Entry Point**: [main.py](main.py)

**Workflow Summary**: Single LLM call → Local Python validation → Execution → Optional verification

**Use Cases**:
- Automated testing of robot capabilities
- Benchmarking task completion
- Batch processing of predefined tasks
- Research experiments requiring reproducibility

**Key Features**:
- No human intervention required during execution
- Python-based validation of plans
- Optional post-execution verification with vision
- Comprehensive logging of all steps

### 2. Console Interactive Mode

**Purpose**: Terminal-based conversational control

**Entry Point**: [main_console.py](main_console.py)

**Workflow Summary**: User message → Claude with tools → Tool execution → Response

**Use Cases**:
- Direct robot control via command line
- Debugging and testing
- Headless server environments
- Quick experimentation

**Key Features**:
- Real-time physics simulation during input
- Rich terminal UI with colors and tables
- Verbose mode for detailed tool execution logs
- Token usage tracking

### 3. Streamlit Interactive Mode

**Purpose**: Web-based UI for conversational control

**Entry Point**: [main_interactive.py](main_interactive.py)

**Workflow Summary**: User message → Claude with tools → Tool execution → Response

**Use Cases**:
- User-friendly interface for non-technical users
- Visual feedback with panorama images
- Demonstrations and presentations
- Remote control via web browser

**Key Features**:
- Beautiful web UI with chat interface
- Real-time scene information in sidebar
- Inline panorama display
- Expandable tool execution details

---

## 📖 Detailed Workflow Explanations

### Batch Mode: Two-Stage LLM Workflow

Batch mode uses a sophisticated two-stage LLM workflow where **two separate API calls work together** to solve a task:

#### Stage 1: Planning (LLMValidator)

**Purpose**: Generate a validated execution plan from the task description

**File**: [src/llm_validator.py](src/llm_validator.py)

**Process**:

1. **Scene Data Preparation**
   - Extract all object positions from the simulation
   - Build detailed object information (positions, dimensions, colors)
   - Example:
     ```
     - red_cube:
       - Position: [0.300, 0.300, 0.050] meters
       - Dimensions: [0.100, 0.100, 0.100] meters
       - Color: RGBA(1.0, 0.0, 0.0, 1.0) [Red]
     ```

2. **LLM Planning Call** (First API Call)
   - **System Prompt**: [prompts/simplified_system_prompt.txt](prompts/simplified_system_prompt.txt)
     - Contains comprehensive instructions on how to plan robot tasks
     - Includes 5 detailed examples of valid plans
     - Specifies available commands: `pick_up`, `place`, `place_on`, `rotate_gripper_90`, `reset_gripper_orientation`
     - Defines constraints (e.g., ground placements must have z=0)
   - **User Prompt**: [prompts/simplified_user_prompt.txt](prompts/simplified_user_prompt.txt)
     - Task description from scene YAML
     - Current object positions and dimensions
   - **Output**: JSON plan with reasoning and commands
     ```json
     {
       "reasoning": "I will pick up the red cube and place it on the blue cube...",
       "commands": [
         {"action": "pick_up", "object_name": "red_cube"},
         {"action": "place_on", "target_object": "blue_cube"}
       ]
     }
     ```

3. **Local Python Validation**
   - **Method**: `LLMValidator._validate_plan_locally()` at [src/llm_validator.py:198](src/llm_validator.py#L198)
   - **Checks**:
     - JSON structure (has `reasoning` and `commands` fields)
     - Valid action names
     - Logical sequence (can't place without picking first)
     - Physical constraints (ground placement z=0)
     - Object name validity
   - **Output**: `(is_valid, errors_list)`

4. **Result**
   - If valid: Plan is passed to execution stage
   - If invalid: Execution aborts with detailed error messages

#### Stage 2: Verification (Optional, LLMValidator)

**Purpose**: Verify task completion after execution

**Configuration**: Controlled by `ENABLE_VERIFICATION` in [src/config.py](src/config.py)

**Process**:

1. **Post-Execution Panorama Capture**
   - Captures 5-view panorama of final scene state
   - Saved to `images/post_execution_*.jpg`

2. **LLM Verification Call** (Second API Call)
   - **System Prompt**: [prompts/verification_system_prompt.txt](prompts/verification_system_prompt.txt)
     - Instructions on how to verify task completion
     - Guidance on comparing actual vs expected states
   - **User Prompt**: [prompts/verification_user_prompt.txt](prompts/verification_user_prompt.txt)
     - Original task description
     - Executed plan
     - Execution status (success/failed)
     - Current object positions
   - **Visual Input**: Post-execution panorama image (base64 encoded)
   - **Output**: JSON verification result
     ```json
     {
       "task_satisfied": true,
       "reasoning": "The red cube is now on top of the blue cube as required",
       "actual_state": "Red cube at [0.3, 0.3, 0.15], on blue cube",
       "expected_state": "Red cube stacked on blue cube",
       "discrepancies": null
     }
     ```

3. **Result**
   - Logs whether task was successfully completed
   - Identifies discrepancies between expected and actual state
   - Provides human-readable explanation

#### How the Two LLM Calls Work Together

```
┌─────────────────────────────────────────────────────────────────┐
│                        BATCH MODE WORKFLOW                      │
└─────────────────────────────────────────────────────────────────┘

Input: Task Description from Scene YAML
  │
  ├─► [LLM Call #1: PLANNING]
  │   ├─ System: How to plan robot tasks
  │   ├─ User: Task + Object positions
  │   └─► Output: Plan JSON {reasoning, commands}
  │
  ├─► [Local Python Validation]
  │   ├─ Check: JSON structure
  │   ├─ Check: Valid actions
  │   ├─ Check: Logical sequence
  │   └─► Output: (is_valid, errors)
  │
  ├─► [If Valid: Execute Plan]
  │   ├─ For each command in plan:
  │   │   └─► Call RobotController method
  │   └─► Output: Execution result
  │
  └─► [LLM Call #2: VERIFICATION] (optional)
      ├─ System: How to verify completion
      ├─ User: Task + Plan + Positions
      ├─ Vision: Post-execution panorama
      └─► Output: Verification JSON
          ├─ task_satisfied: bool
          ├─ reasoning: str
          ├─ actual_state: str
          └─ expected_state: str
```

**Key Insight**: The two LLM calls serve different purposes:
- **Call #1 (Planning)**: Transforms high-level task → low-level command sequence
- **Call #2 (Verification)**: Compares actual outcome → expected outcome

They work together to provide:
1. **Intelligent planning**: Claude reasons about object positions and generates optimal sequences
2. **Safety validation**: Python checks prevent impossible/unsafe operations
3. **Outcome verification**: Claude analyzes the final state to confirm task success

### Interactive Modes: Tool-Based Conversation Workflow

Both console and Streamlit modes use the same underlying architecture with tool calling:

#### Architecture

**File**: [src/interactive_llm_controller.py](src/interactive_llm_controller.py)

**Key Difference from Batch**: Instead of generating a complete plan upfront, Claude decides which tools to use **in real-time** based on conversation context.

#### Available Tools (13 Total)

**Query Tools** (non-destructive, information gathering):
1. `get_gripper_position()` - Current end-effector position
2. `get_gripper_state()` - Open/closed state
3. `get_object_position(object_name)` - Specific object position
4. `get_all_objects()` - List all objects with positions and dimensions
5. `get_panorama()` - Capture 5-view panorama (returned as base64)

**Action Tools** (destructive, modify scene):
6. `move_gripper(x, y, z)` - Standard IK-based movement
7. `move_gripper_smooth(x, y, z)` - Smooth linear interpolation movement
8. `open_gripper()` - Open gripper
9. `close_gripper()` - Close gripper
10. `rotate_gripper_90()` - Rotate gripper 90°
11. `reset_gripper_orientation()` - Reset orientation to default
12. `pick_up_object(object_name)` - Pick up from above
13. `place_object(x, y, z)` - Place at coordinates
14. `place_on_object(target_object)` - Place on another object

#### Workflow: Message → Tools → Response Loop

```
┌─────────────────────────────────────────────────────────────────┐
│                   INTERACTIVE MODE WORKFLOW                     │
└─────────────────────────────────────────────────────────────────┘

User types: "Stack the red cube on the blue cube"
  │
  ├─► [LLM API Call #1]
  │   ├─ System: Interactive control instructions + tool definitions
  │   ├─ Messages: Full conversation history + new user message
  │   └─► Claude decides to use tools:
  │       ├─ tool_use: get_all_objects()
  │       └─ (waits for tool results)
  │
  ├─► [Tool Execution #1]
  │   ├─ Execute: get_all_objects()
  │   └─► Returns: {"objects": [...positions...], "message": "..."}
  │
  ├─► [LLM API Call #2]
  │   ├─ Messages: History + tool results
  │   └─► Claude decides to use more tools:
  │       ├─ tool_use: pick_up_object("red_cube")
  │       └─ (waits for tool results)
  │
  ├─► [Tool Execution #2]
  │   ├─ Execute: pick_up_object("red_cube")
  │   │   ├─ Opens gripper
  │   │   ├─ Moves to object
  │   │   ├─ Closes gripper
  │   │   └─ Lifts up
  │   └─► Returns: {"success": true, "message": "Successfully picked up red_cube"}
  │
  ├─► [LLM API Call #3]
  │   ├─ Messages: History + tool results
  │   └─► Claude decides to use more tools:
  │       ├─ tool_use: place_on_object("blue_cube")
  │       └─ (waits for tool results)
  │
  ├─► [Tool Execution #3]
  │   ├─ Execute: place_on_object("blue_cube")
  │   │   ├─ Gets blue_cube position
  │   │   ├─ Calculates placement height
  │   │   ├─ Moves to position
  │   │   ├─ Opens gripper
  │   │   └─ Retracts
  │   └─► Returns: {"success": true, "message": "Successfully placed..."}
  │
  ├─► [LLM API Call #4]
  │   ├─ Messages: History + tool results
  │   └─► Claude responds with text (no more tools):
  │       "I've successfully stacked the red cube on the blue cube!"
  │
  └─► Display response to user
```

#### Multi-Turn Tool Calling

The interactive mode supports **multi-turn tool calling**, where Claude can:
1. Request multiple tools in sequence
2. Use results from one tool to decide what to do next
3. Continue until the task is complete
4. Respond with natural language explanation

**Example conversation**:
```
User: "Show me the scene and tell me what's there"

[Turn 1] Claude calls: get_all_objects()
         Result: {"objects": [{"name": "red_cube", ...}, ...]}

[Turn 2] Claude calls: get_panorama()
         Result: {"panorama_base64": "..."}

[Turn 3] Claude responds: "Here's what I see in the scene: [panorama displayed]
         There are 3 objects:
         - red_cube at [0.3, 0.3, 0.05]
         - blue_cube at [0.4, 0.2, 0.05]
         - green_cube at [0.5, 0.3, 0.05]"
```

#### Console Mode Specifics

**File**: [main_console.py](main_console.py)

**Key Feature**: Physics runs continuously during user input

```python
def get_input_with_simulation():
    """Get user input while continuously stepping the simulation."""
    while True:
        # Check if input is available (non-blocking)
        readable, _, _ = select.select([sys.stdin], [], [], 0)

        if readable:
            # Read user input
            return sys.stdin.readline().strip()
        else:
            # No input available, step simulation
            p.stepSimulation()
            time.sleep(TIME_STEP)
```

This ensures the robot and objects continue to be affected by physics (gravity, contacts) while the user is typing.

**Special Commands**:
- `/help` - Show available commands
- `/status` - Display scene info, objects, gripper state, token usage
- `/quit` - Exit console
- `/clear` - Clear conversation history
- `/verbose` - Toggle detailed tool execution logs

#### Streamlit Mode Specifics

**File**: [main_interactive.py](main_interactive.py)

**Key Feature**: Cached simulation initialization

```python
@st.cache_resource
def initialize_simulation():
    """Initialize PyBullet simulation and all components (cached)."""
    # ... initialization code ...
```

Streamlit's caching ensures the simulation is only initialized once per session, improving performance.

**UI Features**:
- **Main Chat Area**: Conversation with Claude
- **Sidebar**: Real-time scene information
  - Objects list with positions
  - Gripper state
  - Token usage statistics
  - Example commands
- **Inline Images**: Panoramas displayed directly in chat
- **Tool Details**: Expandable sections showing tool inputs/outputs

---

## 🔍 RAG Pipeline

The system includes a **Retrieval-Augmented Generation (RAG)** pipeline that learns from past successful task executions to improve future performance.

### Overview

The RAG pipeline enables the robot to learn from experience by:
1. **Recording** successful task executions to a database
2. **Retrieving** similar past tasks when handling new requests
3. **Augmenting** prompts with relevant examples

```
┌─────────────────────────────────────────────────────────────────┐
│                        RAG PIPELINE                             │
└─────────────────────────────────────────────────────────────────┘

User Request: "Stack the red cube on the blue cube"
  │
  ├─► [1. Semantic Search]
  │   ├─ Generate embedding for user request
  │   ├─ Query ChromaDB for similar tasks
  │   └─► Find: "Place red block on blue block" (similarity: 0.92)
  │
  ├─► [2. Retrieve Full Context]
  │   ├─ Fetch execution details from SQLite
  │   ├─ Get plan reasoning and commands
  │   └─► Extract: {reasoning, commands, object_config}
  │
  ├─► [3. Format RAG Context]
  │   ├─ Apply rag_context_template.txt
  │   ├─ Format examples with task, reasoning, plan
  │   └─► Generate context string
  │
  └─► [4. Augment Prompt]
      ├─ Inject RAG context into system/user prompt
      └─► LLM receives: original prompt + similar examples
```

### Components

#### TaskStore ([src/task_store.py](src/task_store.py))

The TaskStore handles all database operations:

- **Recording**: Stores task executions with full context
- **Querying**: Finds similar tasks via semantic search
- **Hybrid Storage**: SQLite for structured data, ChromaDB for vectors

```python
# Record a successful execution
task_store.record_execution(
    scene_config=scene,
    task_description="Stack red on blue",
    plan={"reasoning": "...", "commands": [...]},
    execution_result={"status": "success", "steps_completed": 2},
    verification_result={"task_satisfied": True}
)

# Find similar past tasks
similar = task_store.find_similar_tasks(
    query="Put the red cube on top of blue",
    n_results=3,
    success_only=True
)
```

#### RAGRetriever ([src/rag_retriever.py](src/rag_retriever.py))

The RAGRetriever formats retrieved examples for prompt injection:

```python
# Get formatted RAG context for a user message
rag = RAGRetriever()
context = rag.get_rag_context_for_message(
    user_message="Stack the cubes",
    current_objects=["red_cube", "blue_cube"]
)
# Returns formatted examples from past successful tasks
```

### Configuration

RAG settings in [src/config.py](src/config.py):

```python
# Enable/disable RAG
ENABLE_RAG = True

# Number of similar examples to retrieve
RAG_NUM_EXAMPLES = 3

# Embedding model for semantic search
RAG_EMBEDDING_MODEL = 'all-MiniLM-L6-v2'

# Similarity threshold (0-1, lower = more similar in distance space)
RAG_SIMILARITY_THRESHOLD = 0.7

# Template file for formatting RAG context
RAG_CONTEXT_TEMPLATE_FILE = 'rag_context_template.txt'
```

### RAG Context Template

The template at [prompts/rag_context_template.txt](prompts/rag_context_template.txt) formats retrieved examples:

```
## Relevant Past Successful Tasks

The following are examples of similar tasks that were successfully completed.
Use these as guidance for your approach.

{examples}

---

**Note:** These are reference examples. Adapt to the current scene layout.
```

### How RAG Improves Performance

1. **Faster Planning**: Model sees proven solutions for similar tasks
2. **Fewer Errors**: Learn from past successes, avoid repeated mistakes
3. **Scene-Agnostic**: Relative positions allow transfer across different scenes
4. **Continuous Learning**: Each successful execution improves future performance

---

## 🗄 Database Structure

The system uses a hybrid database approach combining **SQLite** for structured data and **ChromaDB** for vector embeddings.

### Database Files

Located in the `data/` folder:

```
data/
├── task_history.db     # SQLite database
└── chroma/             # ChromaDB vector store
    ├── chroma.sqlite3
    └── ...
```

### SQLite Schema

#### task_executions Table

Stores the main task execution records:

| Column | Type | Description |
|--------|------|-------------|
| `id` | INTEGER | Primary key, auto-increment |
| `timestamp` | DATETIME | When the execution occurred |
| `task_description` | TEXT | Natural language task description |
| `plan_reasoning` | TEXT | LLM's reasoning for the plan |
| `plan_commands` | JSON | Array of executed commands |
| `execution_status` | TEXT | 'success' or 'failed' |
| `steps_completed` | INTEGER | Number of steps completed |
| `task_satisfied` | BOOLEAN | Whether verification passed |
| `verification_reasoning` | TEXT | LLM verification explanation |
| `actual_state` | TEXT | Actual final state description |
| `expected_state` | TEXT | Expected state description |
| `discrepancies` | JSON | Array of noted discrepancies |
| `object_count` | INTEGER | Number of objects in scene |
| `object_types` | TEXT | Comma-separated object types |
| `object_colors` | TEXT | Comma-separated color names |

**Indexes:**
- `idx_task_satisfied` - Fast filtering by success
- `idx_object_count` - Filter by scene complexity
- `idx_object_types` - Filter by object types

#### object_states Table

Stores object configurations for each execution (scene-agnostic via relative positions):

| Column | Type | Description |
|--------|------|-------------|
| `id` | INTEGER | Primary key, auto-increment |
| `execution_id` | INTEGER | Foreign key to task_executions |
| `object_index` | INTEGER | Object order in scene |
| `object_type` | TEXT | Object type (e.g., 'cube') |
| `rel_position_x` | REAL | X position relative to centroid |
| `rel_position_y` | REAL | Y position relative to centroid |
| `rel_position_z` | REAL | Z position relative to centroid |
| `color_name` | TEXT | Human-readable color name |
| `color_r` | REAL | Red component (0-1) |
| `color_g` | REAL | Green component (0-1) |
| `color_b` | REAL | Blue component (0-1) |
| `color_a` | REAL | Alpha component (0-1) |
| `scale` | REAL | Object scale factor |

**Scene-Agnostic Design**: Object positions are stored relative to the scene centroid, enabling similarity matching across different absolute positions.

### ChromaDB Collection

The `task_embeddings` collection stores vector embeddings for semantic search:

| Field | Description |
|-------|-------------|
| `id` | Format: `exec_{execution_id}` |
| `document` | Task description text |
| `embedding` | Auto-generated vector embedding |
| `metadata.execution_id` | Link to SQLite record |
| `metadata.task_satisfied` | Success flag for filtering |
| `metadata.object_types` | Object types string |
| `metadata.object_colors` | Object colors string |
| `metadata.object_count` | Number of objects |

**Settings:**
- Embedding function: ChromaDB default (no external dependencies)
- Distance metric: Cosine similarity (`hnsw:space: cosine`)

### Inspecting the Database

Use the helper script to inspect database contents:

```bash
python helpers/sqlite_inspect.py
# Or specify a different database:
python helpers/sqlite_inspect.py data/task_history.db
```

**Example output:**
```
Database: data/task_history.db
Tables: ['task_executions', 'object_states']

--- task_executions (3 rows shown) ---
(1, '2025-11-28 10:30:00', 'Stack red on blue', 'I will pick up...', '[{"action": "pick_up"...}]', 'success', 2, 1, 'Task completed...', 'Red cube on blue', 'Red cube on blue', None, 3, 'cube', 'blue,green,red')

--- object_states (9 rows shown) ---
(1, 1, 0, 'cube', -0.05, 0.1, 0.0, 'red', 1.0, 0.0, 0.0, 1.0, 1.0)
...
```

### Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                      DATA FLOW                                  │
└─────────────────────────────────────────────────────────────────┘

[Batch Mode Execution]
       │
       ├─► Task completes successfully
       │
       ├─► TaskStore.record_execution()
       │   │
       │   ├─► SQLite: INSERT INTO task_executions
       │   │   └─► INSERT INTO object_states (for each object)
       │   │
       │   └─► ChromaDB: collection.add()
       │       └─► Auto-generate embedding from task_description
       │
       └─► Execution ID returned

[Interactive Mode Query]
       │
       ├─► User sends message
       │
       ├─► RAGRetriever.get_rag_context_for_message()
       │   │
       │   ├─► ChromaDB: collection.query()
       │   │   └─► Semantic search with user message
       │   │
       │   ├─► SQLite: SELECT * FROM task_executions WHERE id IN (...)
       │   │   └─► Fetch full execution details
       │   │
       │   └─► Format examples using template
       │
       └─► RAG context injected into prompt
```

---

## 🎬 Scene System

### Scene File Structure

Scenes are defined in YAML files in the [scenes/](scenes/) directory:

```yaml
metadata:
  name: "Example Stacking Scene"
  description: "Three cubes arranged in a line for simple stacking"

objects:
  - name: "red_cube"
    type: "cube"
    position: [0.35, 0.35, 0.05]  # [x, y, z] in meters
    color: [1, 0, 0, 1]           # RGBA (0-1 range)
    scale: 1.0

  - name: "green_cube"
    type: "cube"
    position: [0.4, 0.4, 0.05]
    color: [0, 1, 0, 1]
    scale: 1.0

  - name: "yellow_cube"
    type: "cube"
    position: [0.45, 0.45, 0.05]
    color: [1, 1, 0, 1]
    scale: 1.0

task:
  description: "Stack all cubes on top of each other in this order from bottom to top: red cube, green cube, yellow cube"
```

### Robot Workspace Limits

The Franka Panda robot can only reach objects within these bounds:

- **X-axis**: -0.5 to 0.7 meters
- **Y-axis**: -0.7 to 0.7 meters
- **Z-axis**: 0 to 0.8 meters

Objects placed outside these limits cannot be manipulated.

### Creating Custom Scenes

1. Copy an existing scene:
   ```bash
   cp scenes/default.yaml scenes/my_scene.yaml
   ```

2. Edit object positions, colors, and task:
   ```yaml
   objects:
     - name: "purple_cube"
       type: "cube"
       position: [0.5, -0.3, 0.05]
       color: [0.5, 0.0, 0.5, 1.0]  # Purple
       scale: 1.0

   task:
     description: "Arrange cubes in a line from left to right"
   ```

3. Run with your scene:
   ```bash
   python main.py --scene my_scene
   ```

### Available Scenes

List all available scenes:
```bash
ls scenes/*.yaml
```

Current scenes:
- `default.yaml` - Three cubes for stacking (red, green, yellow)
- `scene_01.yaml` through `scene_07.yaml` - Various cube configurations and tasks

---

## ⚙️ Configuration

All configuration is centralized in [src/config.py](src/config.py).

### Key Configuration Parameters

#### Robot Control

```python
# Movement precision
LINEAR_MOVEMENT_SPEED = 0.1          # m/s for smooth movements
THRESHOLD_PRECISE = 0.001            # Position accuracy (meters)
THRESHOLD_CLOSE_TARGET = 0.01        # Rough positioning threshold

# Physics stabilization
STABILIZATION_LOOP_STEPS = 2500      # Physics steps before starting
TIME_STEP = 0.01                     # Physics simulation timestep
```

#### LLM Settings

```python
# Model selection
ANTHROPIC_MODEL = "claude-sonnet-4-5-20250929"  # From .env file
VALIDATION_MODEL = "claude-sonnet-4-5-20250929"

# Token limits
INTERACTIVE_MAX_TOKENS = 4096        # Max tokens per interactive response
VERIFICATION_MAX_TOKENS = 2048       # Max tokens for verification

# Features
ENABLE_PROMPT_CACHING = True         # Use Anthropic prompt caching
ENABLE_VERIFICATION = True           # Enable post-execution verification
```

#### Camera Settings

```python
# Panorama generation
CAMERA_IMAGE_WIDTH = 640             # Resolution (pixels)
CAMERA_IMAGE_HEIGHT = 480
CAMERA_YAW_ANGLES = [0, 90, 180, 270, 0]  # 5 views: 4 sides + top

# Image compression (affects token usage!)
PANORAMA_FORMAT = 'JPEG'             # JPEG or PNG
PANORAMA_QUALITY = 75                # JPEG quality (1-100)
```

#### Logging

```python
# Log files
LOGS_FOLDER = 'logs'
LOG_API_CALLS = True                 # Detailed API logging

# Session naming
LOG_SESSION_NAME = 'batch'           # Prefix for log files
```

---

## 💾 Prompt Caching Strategy

The system uses Anthropic's [Prompt Caching](https://docs.anthropic.com/en/docs/build-with-claude/prompt-caching) to reduce API costs by up to **90%**.

### How It Works

Prompt caching allows frequently-reused content to be cached on Anthropic's servers:

```python
# Mark content for caching with cache_control
system_with_cache = [
    {
        "type": "text",
        "text": self.system_prompt,
        "cache_control": {"type": "ephemeral"}  # This will be cached!
    }
]
```

Once cached, subsequent requests using the same content:
- **Read from cache**: ~90% cost reduction for cached tokens
- **Write to cache**: Only new content is processed at full price

### Batch Mode Caching

**Planning Stage**:
- System prompt (1500-2000 tokens) cached on first call
- Subsequent scenes reuse cached system prompt
- Savings: ~1500 tokens per task

**Verification Stage** (if enabled):
- System prompt cached
- **Panorama image** cached (~5000-8000 tokens)
- Massive savings on repeated verification

**Example**:
```
First task:
  Input: 250 tokens
  Cache creation: 1842 tokens (system prompt)
  Output: 156 tokens

Second task:
  Input: 300 tokens
  Cache read: 1842 tokens (saved ~90% cost!)
  Output: 189 tokens
```

### Interactive Mode Caching

**System Prompt + Tools**:
- System prompt + all 14 tool definitions cached
- Reused for entire conversation
- Savings: ~1500 tokens per message after first

**Example 10-Message Conversation**:
```
Message 1:
  Cache creation: 1500 tokens
  Processing: 200 tokens

Messages 2-10:
  Cache read: 1500 tokens × 9 = 13,500 tokens
  Processing: ~200 tokens each

Total savings: ~12,000 tokens × 0.9 = significant cost reduction
```

### Configuration

Enable/disable caching:
```python
# In src/config.py
ENABLE_PROMPT_CACHING = True  # Set to False to disable

# Reduce token usage of images
PANORAMA_QUALITY = 50  # Lower quality = fewer tokens
```

### Monitoring Cache Performance

Check logs for cache usage:
```bash
tail -f logs/api_calls_*.log
```

Look for:
```
[LLM] Request completed
  Cache creation tokens: 1842  # New content cached
  Cache read tokens: 0         # Nothing read from cache

[LLM] Request completed (next request)
  Cache creation tokens: 0     # Nothing new to cache
  Cache read tokens: 1842      # Reused cached content!
```

---

## 📊 Logging System

The system provides comprehensive logging at multiple levels.

### Log Files

#### RAG Log
**File**: `logs/rag_queries.log`

**Contents**:
- RAG queries (user messages sent to similarity search)
- Current objects context
- Retrieved examples with task descriptions, object info, and commands
- Formatted RAG context injected into prompts

**Example**:
```
================================================================================
RAG QUERY
--------------------------------------------------------------------------------
Query: Stack the red cube on the blue cube
Current objects: red_cube, blue_cube, green_cube
--------------------------------------------------------------------------------
Results: 2 example(s) found
--------------------------------------------------------------------------------
Example 1:
  Task: Place red block on blue block
  Objects: 3 (cube)
  Colors: blue,green,red
  Reasoning: I will pick up the red cube and place it on top of...
  Commands: 2 command(s)
--------------------------------------------------------------------------------
FORMATTED CONTEXT:
## Relevant Past Successful Tasks
...
================================================================================
```

#### Application Log
**File**: `logs/app_YYYYMMDD_HHMMSS.log`

**Contents**:
- Simulation initialization
- Object loading
- Robot actions (pick, place, move)
- Task execution status
- Errors and warnings

**Example**:
```
2025-11-24 14:30:15 [INFO] Scene: Default Scene - Simple cube manipulation
2025-11-24 14:30:15 [INFO] Objects in scene: red_cube, blue_cube, green_cube
2025-11-24 14:30:16 [INFO] Loaded cube: red_cube at [0.3, 0.3, 0.05]
2025-11-24 14:30:20 [INFO] Executing command: pick_up(red_cube)
2025-11-24 14:30:22 [INFO] Successfully picked up red_cube
```

#### API Call Log
**File**: `logs/api_calls_YYYYMMDD_HHMMSS.log`

**Contents**:
- Complete LLM request/response traces
- Token usage (input, output, cache)
- System prompts
- User prompts
- Model responses
- Tool calls and results

**Example**:
```
========================================
[2025-11-24 14:30:25] LLM REQUEST
Stage: SIMPLIFIED_PLANNING
Model: claude-sonnet-4-5-20250929
----------------------------------------
SYSTEM PROMPT:
You are an AI planner for a Franka Panda robot...
----------------------------------------
USER PROMPT:
Task: Stack the red cube on the blue cube
Objects:
- red_cube: [0.300, 0.300, 0.050]
...
========================================
[2025-11-24 14:30:28] LLM RESPONSE
Tokens: 250 in / 156 out
Cache: 1842 created, 0 read
----------------------------------------
{
  "reasoning": "I will pick up red cube...",
  "commands": [...]
}
```

### Real-Time Monitoring

Watch logs as they're written:
```bash
# Application events
tail -f logs/app_*.log

# API calls and token usage
tail -f logs/api_calls_*.log
```

### Token Usage Tracking

Every API call logs detailed token metrics:
```python
{
    "input_tokens": 250,
    "output_tokens": 156,
    "cache_creation_input_tokens": 1842,  # New cached content
    "cache_read_input_tokens": 0          # Reused cached content
}
```

This allows you to:
- Monitor API costs
- Optimize prompt caching
- Debug unexpected token usage
- Track cache hit rates

---

## 🛠 Development Guide

### Adding New Robot Actions

To add a new action (e.g., `rotate_object`):

#### 1. Add Method to RobotController

Edit [src/robot_controller.py](src/robot_controller.py):
```python
def rotate_object(self, angle_degrees):
    """Rotate the gripper and held object by specified angle."""
    # Implementation
    pass
```

#### 2. For Batch Mode

**Update LLMController** in [src/llm_controller.py](src/llm_controller.py):
```python
def execute_plan(self, plan, task_description):
    # Add handler for new action
    elif cmd_type == "rotate_object":
        angle = command.get("angle")
        self.robot_controller.rotate_object(angle)
```

**Update System Prompt** in [prompts/simplified_system_prompt.txt](prompts/simplified_system_prompt.txt):
```
Available commands:
...
- rotate_object(angle): Rotate held object by angle degrees
```

#### 3. For Interactive Mode

**Add Tool Definition** in [src/interactive_llm_controller.py](src/interactive_llm_controller.py):
```python
def get_tool_definitions(self):
    return [
        # ... existing tools ...
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

**Add Execution Handler** in same file:
```python
def _execute_tool_impl(self, tool_name, tool_input):
    # ... existing handlers ...
    elif tool_name == "rotate_object":
        angle = tool_input["angle"]
        self.robot_controller.rotate_object(angle)
        return {
            "success": True,
            "message": f"Rotated object by {angle} degrees"
        }
```

### Modifying Prompts

All prompts are in [prompts/](prompts/) directory:

- **[simplified_system_prompt.txt](prompts/simplified_system_prompt.txt)**: Batch planning instructions
- **[simplified_user_prompt.txt](prompts/simplified_user_prompt.txt)**: Batch user prompt template
- **[interactive_system_prompt.txt](prompts/interactive_system_prompt.txt)**: Interactive system instructions
- **[verification_system_prompt.txt](prompts/verification_system_prompt.txt)**: Verification instructions
- **[verification_user_prompt.txt](prompts/verification_user_prompt.txt)**: Verification prompt template

**Tips**:
- Include concrete examples for better LLM performance
- Be explicit about constraints and edge cases
- Use consistent formatting for JSON schemas
- Test prompt changes with multiple scenes

### Adding New Object Types

Currently only cubes are supported. To add spheres:

#### 1. Update ObjectManager

Edit [src/object_manager.py](src/object_manager.py):
```python
def load_sphere(self, name, position, radius, color):
    """Load a sphere into the scene."""
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    collision_shape = p.createCollisionShape(
        shapeType=p.GEOM_SPHERE,
        radius=radius
    )
    obj_id = p.createMultiBody(
        baseMass=0.1,
        baseVisualShapeIndex=visual_shape,
        baseCollisionShapeIndex=collision_shape,
        basePosition=position
    )
    self.objects[name] = obj_id
    return obj_id
```

#### 2. Update SceneLoader

Edit [src/scene_loader.py](src/scene_loader.py):
```python
# Add sphere to object schema validation
# Update object loading logic
```

#### 3. Update Main Scripts

Edit [main.py](main.py), [main_console.py](main_console.py), [main_interactive.py](main_interactive.py):
```python
# Add handling for sphere objects
elif obj.type == 'sphere':
    object_manager.load_sphere(obj.name, obj.position, obj.radius, obj.color)
```

#### 4. Update Prompts

Add sphere-specific handling instructions to prompts.

---

## 🐛 Troubleshooting

### Common Issues

#### Plan Validation Failures

**Symptom**: "VALIDATION FAILED - NO VALID PLAN FOUND"

**Causes**:
- Attempting to place without picking first
- Ground placement with z ≠ 0
- Invalid action names
- Missing required fields

**Solution**:
1. Check validation errors in logs
2. Examine plan JSON structure
3. Review validation logic in [src/llm_validator.py:198](src/llm_validator.py#L198)

#### Robot Cannot Reach Object

**Symptom**: Robot fails to move to object or fails IK

**Causes**:
- Object outside workspace limits
- Invalid coordinates in scene YAML
- Physics not stabilized

**Solution**:
1. Verify object position in workspace bounds:
   ```python
   X: -0.5 to 0.7
   Y: -0.7 to 0.7
   Z: 0 to 0.8
   ```
2. Check scene YAML coordinates
3. Ensure stabilization completed (wait for `STABILIZATION_LOOP_STEPS`)

#### Interactive Mode Not Responding

**Console Mode**:
- Physics runs continuously - don't block with long operations
- Check for input/output deadlock

**Streamlit Mode**:
- Streamlit caches initialization - restart if needed
- Clear cache: `streamlit cache clear`

**Both**:
- Check PyBullet GUI isn't stuck: `pkill -f pybullet`

#### High API Costs

**Solutions**:
1. Enable prompt caching:
   ```python
   ENABLE_PROMPT_CACHING = True
   ```
2. Reduce panorama quality:
   ```python
   PANORAMA_QUALITY = 50  # Lower = fewer tokens
   ```
3. Lower max tokens:
   ```python
   INTERACTIVE_MAX_TOKENS = 2048
   ```
4. Monitor usage:
   ```bash
   tail -f logs/api_calls_*.log
   ```

#### Import Errors

**Symptom**: `ModuleNotFoundError: No module named 'anthropic'`

**Solution**:
```bash
# Ensure virtual environment is activated
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

#### PyBullet GUI Issues

**Symptom**: GUI window doesn't appear or crashes

**Solutions**:
- **Linux**: Install OpenGL dependencies
  ```bash
  sudo apt-get install python3-opengl
  ```
- **macOS**: Update XQuartz
- **Windows/WSL**: Use VcXsrv or X410 for GUI forwarding

### Debug Workflow

1. **Check logs first**:
   ```bash
   tail -f logs/app_*.log
   ```

2. **Enable verbose logging**:
   ```python
   # In src/config.py
   LOG_API_CALLS = True
   ```

3. **View generated panoramas**:
   ```bash
   ls -lh images/
   ```

4. **Monitor token usage**:
   ```bash
   grep "Tokens:" logs/api_calls_*.log
   ```

5. **Test with simple scene**:
   ```bash
   python main.py --scene default
   ```

---

## 📝 Important Notes

- **Simplified Workflow**: The codebase uses a simplified single-call planning workflow with local Python validation. Any references to a "5-stage validation workflow" in external docs are outdated.

- **Vision Analysis**: Batch mode does NOT use vision for planning - plans are generated from object position data only. Panoramas are only captured for post-execution verification.

- **Prompt Caching**: System prompts and images are marked with `cache_control: {"type": "ephemeral"}` for Anthropic's caching system.

- **Object Types**: Only cubes are currently supported. URDF/mesh loading for other shapes would require extension.

- **Physics Stabilization**: Always run `STABILIZATION_LOOP_STEPS` before capturing panoramas or executing plans to ensure stable physics state.

- **Coordinate System**: PyBullet uses right-handed coordinate system with Z-up. Ground plane is at z=0.

---

## 📚 Additional Resources

- **PyBullet Documentation**: https://pybullet.org/
- **Anthropic Claude API**: https://docs.anthropic.com/
- **Franka Panda Specs**: https://frankaemika.github.io/
- **Prompt Caching Guide**: https://docs.anthropic.com/en/docs/build-with-claude/prompt-caching

---

## 🙏 Acknowledgments

- Built with [PyBullet](https://pybullet.org/) physics simulation
- Powered by [Anthropic Claude](https://www.anthropic.com/) AI

---

**Happy Robot Controlling! 🤖**
