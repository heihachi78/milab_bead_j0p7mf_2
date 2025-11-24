"""
Configuration file for robot simulation parameters.

This module contains all centralized configuration values for the PyBullet
robotics simulation including robot parameters, physics settings, object
positions, movement thresholds, and camera configuration.
"""

# Robot configuration
END_EFFECTOR_INDEX = 11
NUM_ARM_JOINTS = 7

# Joint limits for Franka Panda arm
LOWER_LIMITS = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
UPPER_LIMITS = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
JOINT_RANGES = [5.8, 3.5, 5.8, 3.0, 5.8, 3.8, 5.8]
REST_POSES = [0, 0, 0, -1.5, 0, 1.5, 0]
JOINT_DAMPING = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# Gripper joint indices
FINGER_JOINT1_INDEX = 9
FINGER_JOINT2_INDEX = 10

# Gripper positions
GRIPPER_OPEN_POSITION = 0.04
GRIPPER_CLOSED_POSITION = 0

# Physics settings
GRAVITY = -9.81

# Motor control settings
ARM_MOTOR_FORCE = 500
GRIPPER_MOTOR_FORCE = 75.0

# Simulation timing
STABILIZATION_STEPS = 100
GRIPPER_MOVEMENT_STEPS = 120
TIME_STEP = 0.01

# Initial conditions
INITIAL_TIME = 0.0
INITIAL_PREV_POSE = [0, 0, 0]
INITIAL_HAS_PREV_POSE = 0

# Simulation settings
USE_NULL_SPACE = 1
USE_ORIENTATION = 1
USE_SIMULATION = 1
USE_REAL_TIME_SIMULATION = 0
IK_SOLVER = 2
TRAIL_DURATION = 15

# Scene configuration
DEFAULT_SCENE = "default"  # Default scene to load if not specified via CLI

# Movement parameters
MAX_ITERATIONS = 10000
DISTANCE_CHANGE_THRESHOLD = 0.0001
LINEAR_MOVEMENT_SPEED = 0.1  # meters per second for smooth straight-line movement (matches ex.py: t/10 with TIME_STEP=0.01)

# IK solver parameters
IK_MAX_ITERATIONS = 1000
IK_RESIDUAL_THRESHOLD = 0.001
POSITION_GAIN = 0.03
POSITION_GAIN_SMOOTH = 0.3
VELOCITY_GAIN = 1
TARGET_VELOCITY = 0

# Pick and place offsets
OVER_TARGET_Z = 0.5

PICK_CLOSE_OFFSET = 0.05
PICK_TARGET_OFFSET = 0.0

PLACE_CLOSE_OFFSET = 0.075
PLACE_TARGET_OFFSET = 0.025

PLACE_ON_CLOSE_OFFSET = 0.1
PLACE_ON_TARGET_OFFSET = 0.05

# Threshold distances
THRESHOLD_OVER_TARGET = 0.05
THRESHOLD_CLOSE_TARGET = 0.01
THRESHOLD_PRECISE_STRICT = 0.0005
THRESHOLD_PRECISE = 0.001

# Orientation threshold (radians) - for checking orientation convergence
# This is the maximum allowed angular difference in quaternion space
ORIENTATION_THRESHOLD = 0.01  # approximately 0.57 degrees

# Main loop parameters
STABILIZATION_LOOP_STEPS = 2500

# Base positions
PLANE_POSITION = [0, 0, 0]
ROBOT_BASE_POSITION = [0, 0, 0]
ROBOT_BASE_ORIENTATION = [0, 0, 0, 1]

# Link index for gripper state
GRIPPER_LINK_INDEX = -1

# Debug line colors
DEBUG_LINE_COLOR_1 = [0, 0, 0.3]
DEBUG_LINE_COLOR_2 = [1, 0, 0]
DEBUG_LINE_WIDTH = 1

# Camera configuration
CAMERA_TARGET_POSITION = [0, 0, 0.5]  # Center of workspace
CAMERA_DISTANCE = 1.25  # Distance from target
CAMERA_IMAGE_WIDTH = 640
CAMERA_IMAGE_HEIGHT = 480
CAMERA_FOV = 60  # Field of view in degrees
CAMERA_NEAR_PLANE = 0.01
CAMERA_FAR_PLANE = 5.0

# Camera positions (5 directions: front, right, back, left, top)
CAMERA_YAW_ANGLES = [0, 90, 180, 270, 0]  # Azimuth angles
CAMERA_PITCH_ANGLES = [0, 0, 0, 0, -89]  # Elevation angles (-89 for near-top view)

# Panorama settings
IMAGES_FOLDER = 'images'
PANORAMA_FORMAT = 'JPEG'  # 'JPEG' or 'PNG' - JPEG uses less tokens for LLM API calls
PANORAMA_QUALITY = 75  # JPEG quality (1-100, higher = better quality but larger file)

# LLM Planning settings (simplified workflow)
MAX_VALIDATION_ITERATIONS = 3  # DEPRECATED: Not used in simplified workflow (kept for compatibility)
VALIDATION_MODEL = None  # None = use same model as ANTHROPIC_MODEL from .env
SIMPLIFIED_MODE = True  # Always True: Uses simplified single-call workflow with local validation

# Logging settings
LOGS_FOLDER = 'logs'
LOG_SESSION_NAME = None  # None = auto-generate timestamp-based name

# API call logging settings (for interactive mode)
LOG_API_CALLS = True  # Enable/disable detailed API call logging
API_CALLS_LOG_FILE = 'api_calls'  # Filename prefix for API call logs (will be: api_calls_{session_name}.log)

# Interactive mode settings
INTERACTIVE_MAX_TOKENS = 4096  # Max tokens for interactive LLM responses
INTERACTIVE_SYSTEM_PROMPT_FILE = 'interactive_system_prompt.txt'  # System prompt file for interactive mode

# Prompt caching settings (for interactive mode)
ENABLE_PROMPT_CACHING = True  # Enable/disable prompt caching for system prompt and tools
PROMPT_CACHE_TTL = None  # None = 5 minutes (default), "1h" = 1 hour (costs 2x for cache writes)

# Collision prediction settings (new feature)
ENABLE_COLLISION_PREDICTION = True  # Enable/disable collision prediction before executing each step
COLLISION_CHECK_MODEL = None  # None = use same model as ANTHROPIC_MODEL from .env
COLLISION_CHECK_CACHE_TTL = 300  # Cache TTL for collision check prompts (seconds)
