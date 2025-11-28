"""
PyBullet robotics simulation package.

This package provides modules for LLM-controlled robot manipulation:
    - robot_controller: Low-level robot control (IK, gripper, pick/place)
    - llm_controller: Executes validated plans from LLM
    - llm_validator: Generates and validates execution plans
    - interactive_llm_controller: Tool-based conversational control
    - object_manager: Object loading and position queries
    - camera_manager: Multi-view panorama capture
    - scene_loader: YAML scene configuration parsing
    - simulation_state: Simulation time and state tracking
    - config: Centralized configuration parameters
    - logger: Dual logging system (console + file)
"""
