"""
Batch mode execution for LLM-controlled robot tasks.

This script runs the robot simulation in autonomous batch mode, where:
1. A scene is loaded from YAML configuration
2. The LLM generates and validates an execution plan
3. The plan is executed by the robot controller
4. Optional post-execution verification checks task completion

Usage:
    python main.py --scene default
    python main.py --scene scene_01

References:
    https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
    https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py
"""

import pybullet as p
import os
import glob
import argparse
from dotenv import load_dotenv
from anthropic import Anthropic
from src.config import *

# Load environment variables from .env file
load_dotenv()
from src.simulation_state import SimulationState
from src.object_manager import ObjectManager
from src.robot_controller import RobotController
from src.camera_manager import CameraManager
from src.llm_controller import LLMController
from src.llm_validator import LLMValidator
from src.logger import SimulationLogger
from src.scene_loader import load_scene, list_available_scenes, SceneLoadError

# Parse command line arguments
parser = argparse.ArgumentParser(description='PyBullet Franka Panda robotics simulation with LLM control')
parser.add_argument('--scene', type=str, default=DEFAULT_SCENE,
                    help=f'Scene name to load (default: {DEFAULT_SCENE}). Available: {", ".join(list_available_scenes())}')
args = parser.parse_args()

# Load scene configuration
logger = SimulationLogger(log_dir=LOGS_FOLDER, session_name=LOG_SESSION_NAME)
logger.console_info("Initializing simulation...")

try:
    scene = load_scene(args.scene)
    logger.log_scene_loaded(scene.metadata.name, scene.metadata.description, scene.get_object_names())
except SceneLoadError as e:
    logger.log_scene_load_error(args.scene, str(e))
    logger.close()
    exit(1)

# Initialize PyBullet using RobotController's static method
armId = RobotController.initialize_pybullet(logger=logger)
endEffectorIndex = END_EFFECTOR_INDEX

# Initialize simulation components
simulation_state = SimulationState()
object_manager = ObjectManager(logger)
camera_manager = CameraManager(logger)
robot_controller = RobotController(armId, endEffectorIndex, simulation_state, object_manager, camera_manager, logger)

# Stabilize robot
logger.console_info("Stabilizing robot...")
robot_controller.stabilize()

# Clear images folder from previous runs
images_folder = IMAGES_FOLDER
if os.path.exists(images_folder):
    file_count = len(glob.glob(os.path.join(images_folder, '*')))
    for file in glob.glob(os.path.join(images_folder, '*')):
        os.remove(file)
    logger.log_images_folder_cleared(images_folder, file_count)

# Set simulation parameters
p.setRealTimeSimulation(USE_REAL_TIME_SIMULATION)
logger.log_realtime_simulation_set(USE_REAL_TIME_SIMULATION)

# Load objects from scene configuration
logger.console_info("Loading objects into scene...")
for obj in scene.objects:
    if obj.type == 'cube':
        object_manager.load_cube(obj.name, obj.position, obj.color, obj.scale)
        logger.log_object_loaded(obj.type, obj.name, obj.position)
    else:
        logger.log_unknown_object_type(obj.type, obj.name)
logger.log_objects_loaded_count(len(scene.objects))

# Stabilization loop
logger.console_info("Running stabilization loop...")
# First stabilize robot at rest pose with motors active
robot_controller.stabilize()
# Reset gripper orientation (move_to_target_smooth runs simulation internally)
robot_controller.move_to_target([0.25, 0.25, 0.5], THRESHOLD_PRECISE)
robot_controller.reset_orientation()
# Allow system to settle with correct orientation
for i in range(STABILIZATION_LOOP_STEPS):
    p.stepSimulation()
logger.log_stabilization_complete(STABILIZATION_LOOP_STEPS)

# Capture initial panorama after stabilization
logger.console_info("Capturing initial scene panorama...")
camera_manager.capture_and_save_panorama("initial_stabilized")
logger.log_panorama_captured("Initial")

# Initialize LLM controller and validator
logger.console_info("Initializing LLM systems...")

# Create shared Anthropic client (used by both validator and controller)
anthropic_api_key = os.getenv('ANTHROPIC_API_KEY')
if not anthropic_api_key:
    logger.log_api_key_missing()
    raise ValueError("ANTHROPIC_API_KEY is required")

anthropic_client = Anthropic(api_key=anthropic_api_key)

llm_controller = LLMController(object_manager, robot_controller, logger, anthropic_client)
llm_validator = LLMValidator(object_manager, logger, anthropic_client)
logger.log_llm_systems_initialized()

# Main pick and place operations
logger.log_app_simulation_start(simulation_state.t)

# Get task description from scene
task_description = scene.task.description
logger.log_task_description(task_description)

# Generate and validate plan using simplified workflow
logger.console_info("Generating and validating execution plan...")
validated_plan, is_valid, final_critique = llm_validator.get_validated_plan(task_description)

# Check if validation succeeded
if not is_valid:
    commands = validated_plan.get('commands', [])
    logger.log_validation_failed(final_critique, commands)
    logger.log_app_simulation_end(simulation_state.t)
    logger.close()
    p.disconnect()
    exit(1)

# Validate that the plan has commands
commands = validated_plan.get('commands', [])
if not commands or len(commands) == 0:
    logger.log_empty_plan_warning()
    logger.log_app_simulation_end(simulation_state.t)
    logger.close()
    p.disconnect()
    exit(0)

# Execute the validated plan
logger.log_execution_starting(len(commands))

# Execute plan and handle both success and failure cases
execution_result = None
execution_failed_with_exception = False
exception_message = ""

try:
    execution_result = llm_controller.execute_plan(validated_plan, task_description)
except Exception as e:
    # Execution failed with exception - create a failure result
    execution_failed_with_exception = True
    exception_message = str(e)
    execution_result = {
        "status": "failed",
        "reason": "Execution raised exception",
        "details": exception_message,
        "steps_completed": 0
    }
    logger.log_execution_exception(exception_message)

# Check execution result (if didn't fail with exception)
if not execution_failed_with_exception:
    if execution_result["status"] == "failed":
        logger.log_execution_failed(execution_result['reason'], execution_result.get('details'))
    else:
        logger.log_execution_success(execution_result['steps_completed'])

# Post-execution verification (if enabled)
if ENABLE_VERIFICATION and execution_result is not None:
    logger.log_verification_header()

    # Capture post-execution panorama
    logger.console_info("Capturing post-execution panorama...")
    post_execution_panorama = camera_manager.capture_and_save_panorama("post_execution")
    logger.log_verification_panorama_saved(post_execution_panorama)

    # Run verification
    verification_result = llm_validator.verify_task_completion(
        task_description=task_description,
        original_plan=validated_plan,
        execution_result=execution_result,
        panorama_path=post_execution_panorama
    )

    # Display verification results
    logger.log_verification_results(verification_result)

logger.log_app_simulation_end(simulation_state.t)
logger.console_info("Simulation completed successfully")

# Close logger
logger.close()

p.disconnect()
