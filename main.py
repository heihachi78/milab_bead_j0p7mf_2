import pybullet as p
import pybullet_data
from config import *
from simulation_state import SimulationState
from object_manager import ObjectManager
from robot_controller import RobotController
from camera_manager import CameraManager

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

# Set simulation parameters
p.setRealTimeSimulation(USE_REAL_TIME_SIMULATION)

# Load objects
object_manager.load_cube('blue_cube', BLUE_CUBE_POS, BLUE_COLOR)
object_manager.load_cube('red_cube', RED_CUBE_POS, RED_COLOR)
object_manager.load_cube('green_cube', GREEN_CUBE_POS, GREEN_COLOR)

# Stabilization loop
for i in range(STABILIZATION_LOOP_STEPS):
    p.stepSimulation()

# Capture initial panorama after stabilization
camera_manager.capture_and_save_panorama("initial_stabilized")

# Main pick and place operations
print(f"==========================================")
print(f"simulation started t={simulation_state.t}")
print(f"==========================================")

robot_controller.pick_up('blue_cube')
robot_controller.place(object_manager.get_object_center_position('red_cube'))
robot_controller.pick_up('green_cube')
robot_controller.place(object_manager.get_object_center_position('blue_cube'))

print(f"==========================================")
print(f"simulation ended t={simulation_state.t}")
print(f"==========================================")

p.disconnect()
