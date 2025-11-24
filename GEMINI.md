# Project Overview

This project is a Python-based robotics simulation that uses the PyBullet physics engine to control a Franka Panda robot arm. The robot can be controlled through natural language commands processed by the Claude AI model from Anthropic. The project supports three modes of operation: a batch mode for autonomous execution, an interactive console mode, and an interactive web-based mode using Streamlit.

The core of the project lies in its ability to translate high-level natural language tasks (e.g., "stack the red cube on the blue cube") into a series of low-level robot actions. In batch mode, it uses a sophisticated multi-stage validation process to ensure the generated plan is feasible before execution. In interactive modes, it uses a tool-based approach where the LLM can call predefined functions to query the environment and control the robot.

## Main Technologies

*   **Python 3.8+**
*   **PyBullet**: For physics simulation.
*   **Anthropic Claude AI**: For natural language understanding and planning.
*   **Streamlit**: for the interactive web UI.
*   **Rich**: For the interactive console UI.
*   **PyYAML**: For scene configuration.

# Building and Running

## Installation

1.  **Create a virtual environment:**
    ```bash
    python -m venv .venv
    source .venv/bin/activate
    ```

2.  **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

3.  **Set up environment variables:**
    Create a `.env` file in the project root with your Anthropic API key:
    ```
    ANTHROPIC_API_KEY=your_anthropic_api_key_here
    ```

## Running the Project

The project has three execution modes:

1.  **Batch Mode (`main.py`):**
    Executes a task from a scene file autonomously.
    ```bash
    python main.py --scene <scene_name>
    ```
    Example:
    ```bash
    python main.py --scene default
    ```

2.  **Console Interactive Mode (`main_console.py`):**
    A rich text-based interface for controlling the robot.
    ```bash
    python main_console.py --scene <scene_name>
    ```
    Example:
    ```bash
    python main_console.py --scene default
    ```

3.  **Streamlit Interactive Mode (`main_interactive.py`):**
    A web-based graphical interface for controlling the robot.
    ```bash
    streamlit run main_interactive.py
    ```
    Then open your browser to `http://localhost:8501`.

# Development Conventions

*   **Configuration:** All main configuration parameters are centralized in `src/config.py`. This includes robot parameters, physics settings, and LLM settings.
*   **Scenes:** The simulation environment is defined in YAML files located in the `scenes/` directory. Each scene file specifies the objects in the environment and the task to be performed.
*   **Prompts:** The prompts for the LLM are stored in the `prompts/` directory. There are separate prompts for the different stages of the validation process and for the interactive mode.
*   **Logging:** The project has a comprehensive logging system that logs application events, simulation data, and API calls to the `logs/` directory.
*   **Modularity:** The code is organized into modules with clear responsibilities:
    *   `src/robot_controller.py`: Low-level robot control.
    *   `src/llm_validator.py`: Multi-stage plan validation for batch mode.
    *   `src/interactive_llm_controller.py`: Tool-based control for interactive modes.
    *   `src/object_manager.py`: Manages objects in the simulation.
    *   `src/camera_manager.py`: Handles camera and panorama creation.
    *   `src/scene_loader.py`: Loads scene files.
*   **Tool-based LLM Interaction:** The interactive mode heavily relies on a tool-based architecture where the LLM can call Python functions to get information and control the robot. These tools are defined in `src/interactive_llm_controller.py`.
