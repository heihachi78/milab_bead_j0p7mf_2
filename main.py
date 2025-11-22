#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py

import pybullet as p
import pybullet_data
import os
import glob
from src.config import *
from src.simulation_state import SimulationState
from src.object_manager import ObjectManager
from src.robot_controller import RobotController
from src.camera_manager import CameraManager
from src.llm_controller import LLMController
from src.llm_validator import LLMValidator
from src.logger import SimulationLogger

# Initialize logger
logger = SimulationLogger(log_dir=LOGS_FOLDER, session_name=LOG_SESSION_NAME)
logger.console_info("Initializing simulation...")

# Connect to PyBullet
clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
logger.app_logger.info("PyBullet connected successfully")

# Load plane and robot
p.loadURDF("plane.urdf", PLANE_POSITION)
logger.app_logger.info(f"Loaded plane at {PLANE_POSITION}")

armId = p.loadURDF("franka_panda/panda.urdf", ROBOT_BASE_POSITION, useFixedBase=True)
p.resetBasePositionAndOrientation(armId, ROBOT_BASE_POSITION, ROBOT_BASE_ORIENTATION)
endEffectorIndex = END_EFFECTOR_INDEX
numJoints = p.getNumJoints(armId)
logger.app_logger.info(f"Loaded Franka Panda robot with {numJoints} joints")
logger.console_info(f"Robot loaded: {numJoints} joints")

# Set gravity
p.setGravity(0, 0, GRAVITY)
logger.app_logger.info(f"Gravity set to {GRAVITY}")

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

# Load objects
logger.console_info("Loading objects into scene...")
object_manager.load_cube('blue_cube', BLUE_CUBE_POS, BLUE_COLOR)
object_manager.load_cube('red_cube', RED_CUBE_POS, RED_COLOR)
object_manager.load_cube('green_cube', GREEN_CUBE_POS, GREEN_COLOR)
object_manager.load_cube('yellow_cube', YELLOW_CUBE_POS, YELLOW_COLOR)
object_manager.load_cube('purple_cube', PURPLE_CUBE_POS, PURPLE_COLOR)
logger.console_info("All objects loaded successfully")

# Stabilization loop
logger.console_info("Running stabilization loop...")
for i in range(STABILIZATION_LOOP_STEPS):
    p.stepSimulation()
logger.app_logger.info(f"Stabilization loop completed: {STABILIZATION_LOOP_STEPS} steps")

# Capture initial panorama after stabilization
logger.console_info("Capturing initial scene panorama...")
camera_manager.capture_and_save_panorama("initial_stabilized")
logger.console_info("Initial panorama captured")

# Initialize LLM controller and validator
logger.console_info("Initializing LLM systems...")
llm_controller = LLMController(object_manager, robot_controller, logger)
llm_validator = LLMValidator(object_manager, logger)
logger.app_logger.info("LLM controller and validator initialized")

# Main pick and place operations
logger.log_app_simulation_start(simulation_state.t)

# Load task description
task_description = llm_controller.task_description
logger.app_logger.info(f"Task description loaded: {task_description[:100]}...")

# Find the latest panorama
logger.console_info("Finding latest panorama...")
panorama_path = llm_controller._find_latest_panorama()
logger.app_logger.info(f"Using panorama: {panorama_path}")

# Generate and validate plan using LangChain workflow
logger.console_info("Generating and validating execution plan...")
validated_plan = llm_validator.get_validated_plan(task_description, panorama_path)

# Execute the validated plan
logger.console_info("Executing validated plan...")
llm_controller.execute_plan(validated_plan)

'''
robot_controller.pick_up('green_cube')
robot_controller.place([-0.45, 0.45, 0])
robot_controller.pick_up('blue_cube')
robot_controller.place_on('red_cube')
'''

logger.log_app_simulation_end(simulation_state.t)
logger.console_info("Simulation completed successfully")

# Close logger
logger.close()

p.disconnect()
