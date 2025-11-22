#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
#https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py

import pybullet as p
import pybullet_data
import os
import glob
from config import *
from simulation_state import SimulationState
from object_manager import ObjectManager
from robot_controller import RobotController
from camera_manager import CameraManager
from llm_controller import LLMController

# Connect to PyBullet
clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane and robot
p.loadURDF("plane.urdf", PLANE_POSITION)
armId = p.loadURDF("franka_panda/panda.urdf", ROBOT_BASE_POSITION, useFixedBase=True)
p.resetBasePositionAndOrientation(armId, ROBOT_BASE_POSITION, ROBOT_BASE_ORIENTATION)
endEffectorIndex = END_EFFECTOR_INDEX
numJoints = p.getNumJoints(armId)
print(f"Number of joints: {numJoints}")

# Set gravity
p.setGravity(0, 0, GRAVITY)

# Initialize simulation components
simulation_state = SimulationState()
object_manager = ObjectManager()
camera_manager = CameraManager()
robot_controller = RobotController(armId, endEffectorIndex, simulation_state, object_manager, camera_manager)

# Stabilize robot
robot_controller.stabilize()

# Clear images folder from previous runs
images_folder = IMAGES_FOLDER
if os.path.exists(images_folder):
    for file in glob.glob(os.path.join(images_folder, '*')):
        os.remove(file)
    print(f"Cleared {images_folder} folder")

# Set simulation parameters
p.setRealTimeSimulation(USE_REAL_TIME_SIMULATION)

# Load objects
object_manager.load_cube('blue_cube', BLUE_CUBE_POS, BLUE_COLOR)
object_manager.load_cube('red_cube', RED_CUBE_POS, RED_COLOR)
object_manager.load_cube('green_cube', GREEN_CUBE_POS, GREEN_COLOR)
object_manager.load_cube('yellow_cube', YELLOW_CUBE_POS, YELLOW_COLOR)
object_manager.load_cube('purple_cube', PURPLE_CUBE_POS, PURPLE_COLOR)

# Stabilization loop
for i in range(STABILIZATION_LOOP_STEPS):
    p.stepSimulation()

# Capture initial panorama after stabilization
camera_manager.capture_and_save_panorama("initial_stabilized")

# Initialize LLM controller
llm_controller = LLMController(object_manager, robot_controller)

# Main pick and place operations
print(f"==========================================")
print(f"simulation started t={simulation_state.t}")
print(f"==========================================")

# Generate plan using LLM
#plan = llm_controller.generate_plan()

# Execute the plan
#llm_controller.execute_plan(plan)


robot_controller.pick_up('blue_cube')
robot_controller.pick_up('red_cube')
robot_controller.pick_up('green_cube')
robot_controller.place([-0.45, 0.45, 0])


print(f"==========================================")
print(f"simulation ended t={simulation_state.t}")
print(f"==========================================")

p.disconnect()
