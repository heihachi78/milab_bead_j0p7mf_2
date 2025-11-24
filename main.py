#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py

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
    logger.console_info(f"Loaded scene: {scene.metadata.name}")
    logger.app_logger.info(f"Scene: {scene.metadata.name} - {scene.metadata.description}")
    logger.app_logger.info(f"Objects in scene: {', '.join(scene.get_object_names())}")
except SceneLoadError as e:
    print(f"ERROR: Failed to load scene '{args.scene}': {e}")
    logger.app_logger.error(f"Failed to load scene '{args.scene}': {e}")
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
logger.app_logger.info("Simulation components initialized")

# Stabilize robot
logger.console_info("Stabilizing robot...")
robot_controller.stabilize()

# Clear images folder from previous runs
images_folder = IMAGES_FOLDER
if os.path.exists(images_folder):
    file_count = len(glob.glob(os.path.join(images_folder, '*')))
    for file in glob.glob(os.path.join(images_folder, '*')):
        os.remove(file)
    logger.app_logger.info(f"Cleared {file_count} files from {images_folder} folder")

# Set simulation parameters
p.setRealTimeSimulation(USE_REAL_TIME_SIMULATION)
logger.app_logger.info(f"Real-time simulation: {USE_REAL_TIME_SIMULATION}")

# Load objects from scene configuration
logger.console_info("Loading objects into scene...")
for obj in scene.objects:
    if obj.type == 'cube':
        object_manager.load_cube(obj.name, obj.position, obj.color, obj.scale)
        logger.app_logger.info(f"Loaded {obj.type}: {obj.name} at {obj.position}")
    else:
        logger.app_logger.warning(f"Unknown object type '{obj.type}' for object '{obj.name}', skipping")
logger.console_info(f"Loaded {len(scene.objects)} objects successfully")

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
logger.app_logger.info(f"Stabilization loop completed: {STABILIZATION_LOOP_STEPS} steps")

# Capture initial panorama after stabilization
logger.console_info("Capturing initial scene panorama...")
camera_manager.capture_and_save_panorama("initial_stabilized")
logger.console_info("Initial panorama captured")

# Initialize LLM controller and validator
logger.console_info("Initializing LLM systems...")

# Create shared Anthropic client (used by both validator and controller)
anthropic_api_key = os.getenv('ANTHROPIC_API_KEY')
if not anthropic_api_key:
    logger.console_error("ANTHROPIC_API_KEY not found in environment variables")
    raise ValueError("ANTHROPIC_API_KEY is required")

anthropic_client = Anthropic(api_key=anthropic_api_key)

llm_controller = LLMController(object_manager, robot_controller, logger, anthropic_client)
llm_validator = LLMValidator(object_manager, logger, anthropic_client)
logger.app_logger.info("LLM controller and validator initialized")

# Main pick and place operations
logger.log_app_simulation_start(simulation_state.t)

# Get task description from scene
task_description = scene.task.description
logger.app_logger.info(f"Task description: {task_description[:100]}...")

# Find the latest panorama
logger.console_info("Finding latest panorama...")
panorama_path = llm_controller._find_latest_panorama()
logger.app_logger.info(f"Using panorama: {panorama_path}")

# Generate and validate plan using simplified workflow
logger.console_info("Generating and validating execution plan...")
validated_plan, is_valid, final_critique = llm_validator.get_validated_plan(task_description, panorama_path)

# Check if validation succeeded
if not is_valid:
    logger.console_error("=" * 80)
    logger.console_error("VALIDATION FAILED - NO VALID PLAN FOUND")
    logger.console_error("=" * 80)
    logger.console_error("")
    logger.console_error("The LLM could not generate a valid plan.")
    logger.console_error("")

    if final_critique:
        logger.console_error("Validation errors:")
        errors = final_critique.get('errors', [])
        if errors:
            for error in errors:
                logger.console_error(f"  - {error}")
        elif 'error' in final_critique:
            logger.console_error(f"  - {final_critique['error']}")
        logger.console_error("")

    # Check if plan has no commands
    commands = validated_plan.get('commands', [])
    if not commands or len(commands) == 0:
        logger.console_error("The final plan contains NO COMMANDS (empty command list).")
        logger.console_error("This usually means the LLM could not generate a valid command sequence.")
    else:
        logger.console_error(f"The final plan has {len(commands)} command(s) but failed validation checks.")

    logger.console_error("")
    logger.console_error("Possible reasons:")
    logger.console_error("  - The task may be impossible given the current scene configuration")
    logger.console_error("  - Object positions may make the task infeasible")
    logger.console_error("  - The LLM may be confused about the scene state")
    logger.console_error("")
    logger.console_error("ABORTING EXECUTION - No robot operations will be performed.")
    logger.console_error("=" * 80)

    logger.log_app_simulation_end(simulation_state.t)
    logger.close()
    p.disconnect()
    exit(1)

# Validate that the plan has commands
commands = validated_plan.get('commands', [])
if not commands or len(commands) == 0:
    logger.console_warning("=" * 80)
    logger.console_warning("WARNING: Plan contains no commands")
    logger.console_warning("=" * 80)
    logger.console_warning("")
    logger.console_warning("The validated plan has an empty command list.")
    logger.console_warning("This may indicate that:")
    logger.console_warning("  - The scene is already in the desired state")
    logger.console_warning("  - No actions are needed to complete the task")
    logger.console_warning("")
    logger.console_warning("Skipping execution (nothing to do).")
    logger.console_warning("=" * 80)

    logger.log_app_simulation_end(simulation_state.t)
    logger.close()
    p.disconnect()
    exit(0)

# Execute the validated plan
logger.console_info("Executing validated plan...")
logger.console_info(f"Plan contains {len(commands)} command(s)")
execution_result = llm_controller.execute_plan(validated_plan, task_description)

# Check execution result
if execution_result["status"] == "failed":
    logger.console_error("=" * 80)
    logger.console_error("EXECUTION FAILED")
    logger.console_error("=" * 80)
    logger.console_error(f"Reason: {execution_result['reason']}")
    if "details" in execution_result:
        logger.console_error(f"Details: {execution_result['details']}")
    logger.console_error("=" * 80)
else:
    logger.console_success("=" * 80)
    logger.console_success("EXECUTION COMPLETED SUCCESSFULLY")
    logger.console_success(f"Completed {execution_result['steps_completed']} steps")
    logger.console_success("=" * 80)

'''
robot_controller.pick_up('purple_cube')
robot_controller.rotate_orientation_90()
robot_controller.place_on('yellow_cube')
robot_controller.reset_orientation()
robot_controller.pick_up('red_cube')
robot_controller.place_on('green_cube')
'''

logger.log_app_simulation_end(simulation_state.t)
logger.console_info("Simulation completed successfully")

# Close logger
logger.close()

p.disconnect()
