"""
PyBullet GUI server for visual feedback.

This script launches a PyBullet GUI simulation that runs as a shared memory server.
The Streamlit interactive interface (main_interactive.py) can then connect to this
server to control the robot while seeing the visualization.

Usage:
    1. First, run this script in one terminal:
       python main_visual.py

    2. Then, run the Streamlit interface in another terminal:
       streamlit run main_interactive.py
"""

import pybullet as p
import pybullet_data
import sys
import time

from src.config import *
from src.simulation_state import SimulationState
from src.object_manager import ObjectManager
from src.robot_controller import RobotController
from src.camera_manager import CameraManager
from src.logger import SimulationLogger
from src.scene_loader import load_scene, SceneLoadError


def get_scene_name_from_args():
    """Extract scene name from command line arguments."""
    scene_name = DEFAULT_SCENE

    if "--scene" in sys.argv:
        try:
            scene_idx = sys.argv.index("--scene")
            if scene_idx + 1 < len(sys.argv):
                scene_name = sys.argv[scene_idx + 1]
        except (ValueError, IndexError):
            pass

    return scene_name


def main():
    """Main function to run PyBullet GUI server."""

    # Initialize logger
    logger = SimulationLogger(log_dir=LOGS_FOLDER, session_name="visual_server")
    logger.console_info("Starting PyBullet GUI server...")

    # Load scene configuration
    scene_name = get_scene_name_from_args()
    try:
        scene = load_scene(scene_name)
        logger.console_info(f"Loaded scene: {scene.metadata.name}")
        logger.app_logger.info(f"Scene: {scene.metadata.name} - {scene.metadata.description}")
    except SceneLoadError as e:
        print(f"ERROR: Failed to load scene '{scene_name}': {e}")
        logger.app_logger.error(f"Failed to load scene '{scene_name}': {e}")
        logger.close()
        sys.exit(1)

    # Connect to PyBullet in SHARED_MEMORY_SERVER mode
    # This allows other processes to connect via SHARED_MEMORY
    logger.console_info("Connecting to PyBullet in GUI + SHARED_MEMORY_SERVER mode...")
    p.connect(p.GUI_SERVER)

    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    logger.app_logger.info("PyBullet GUI server started successfully")
    logger.console_info("✓ PyBullet GUI server running")

    # Load plane and robot
    p.loadURDF("plane.urdf", PLANE_POSITION)
    logger.app_logger.info(f"Loaded plane at {PLANE_POSITION}")

    armId = p.loadURDF("franka_panda/panda.urdf", ROBOT_BASE_POSITION, useFixedBase=True)
    p.resetBasePositionAndOrientation(armId, ROBOT_BASE_POSITION, ROBOT_BASE_ORIENTATION)
    endEffectorIndex = END_EFFECTOR_INDEX
    numJoints = p.getNumJoints(armId)
    logger.app_logger.info(f"Loaded Franka Panda robot with {numJoints} joints")
    logger.console_info(f"✓ Robot loaded: {numJoints} joints")

    # Set gravity
    p.setGravity(0, 0, GRAVITY)
    logger.app_logger.info(f"Gravity set to {GRAVITY}")

    # Initialize simulation components (for loading objects)
    simulation_state = SimulationState()
    object_manager = ObjectManager(logger)
    camera_manager = CameraManager(logger)
    robot_controller = RobotController(armId, endEffectorIndex, simulation_state, object_manager, camera_manager, logger)
    logger.app_logger.info("Simulation components initialized")

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
    logger.console_info(f"✓ Loaded {len(scene.objects)} objects successfully")

    # Stabilization loop
    logger.console_info("Running stabilization loop...")
    robot_controller.stabilize()
    robot_controller.move_to_target([0.25, 0.25, 0.5], THRESHOLD_PRECISE)
    robot_controller.reset_orientation()
    for i in range(STABILIZATION_LOOP_STEPS):
        p.stepSimulation()
    logger.app_logger.info(f"Stabilization loop completed: {STABILIZATION_LOOP_STEPS} steps")
    logger.console_info("✓ Stabilization complete")

    # Print connection info
    print("\n" + "="*60)
    print("PyBullet GUI Server Ready!")
    print("="*60)
    print(f"Scene: {scene.metadata.name}")
    print(f"Objects: {len(scene.objects)}")
    print("\nNow you can run in another terminal:")
    print("  streamlit run main_interactive.py")
    print("\nThe Streamlit interface will connect to this visualization.")
    print("Press Ctrl+C to stop the server.")
    print("="*60 + "\n")

    # Keep the server running
    try:
        while True:
            # Keep simulation running if real-time mode is disabled
            if not USE_REAL_TIME_SIMULATION:
                p.stepSimulation()
            time.sleep(0.01)  # Small sleep to prevent 100% CPU usage
    except KeyboardInterrupt:
        logger.console_info("\nShutting down PyBullet GUI server...")
        p.disconnect()
        logger.close()
        print("Server stopped.")


if __name__ == "__main__":
    main()
