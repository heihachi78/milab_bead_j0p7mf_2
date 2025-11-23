"""
Interactive mode for robot control using Streamlit chat interface.

This script launches a Streamlit web app that allows users to interact with
the robot simulation through natural language commands via Claude LLM.
"""

import pybullet as p
import pybullet_data
import os
import glob
import sys
import streamlit as st
import base64
from io import BytesIO
from PIL import Image

from src.config import *
from src.simulation_state import SimulationState
from src.object_manager import ObjectManager
from src.robot_controller import RobotController
from src.camera_manager import CameraManager
from src.interactive_llm_controller import InteractiveLLMController
from src.logger import SimulationLogger
from src.scene_loader import load_scene, list_available_scenes, SceneLoadError


# Streamlit page configuration
st.set_page_config(
    page_title="Robot Control Chat",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for better chat appearance
st.markdown("""
<style>
    .main {
        background-color: #0e1117;
    }
    .stChatMessage {
        padding: 1rem;
        border-radius: 0.5rem;
    }
    div[data-testid="stToolbar"] {
        display: none;
    }
</style>
""", unsafe_allow_html=True)


def get_scene_name_from_args():
    """Extract scene name from command line arguments."""
    # Streamlit uses -- to separate its args from script args
    # Usage: streamlit run main_interactive.py -- --scene my_scene
    scene_name = DEFAULT_SCENE

    if "--scene" in sys.argv:
        try:
            scene_idx = sys.argv.index("--scene")
            if scene_idx + 1 < len(sys.argv):
                scene_name = sys.argv[scene_idx + 1]
        except (ValueError, IndexError):
            pass

    return scene_name


@st.cache_resource
def initialize_simulation():
    """Initialize PyBullet simulation and all components (cached)."""

    # Initialize logger
    logger = SimulationLogger(log_dir=LOGS_FOLDER, session_name=LOG_SESSION_NAME)
    logger.console_info("Initializing interactive simulation...")

    # Load scene configuration
    scene_name = get_scene_name_from_args()
    try:
        scene = load_scene(scene_name)
        logger.console_info(f"Loaded scene: {scene.metadata.name}")
        logger.app_logger.info(f"Scene: {scene.metadata.name} - {scene.metadata.description}")
        logger.app_logger.info(f"Objects in scene: {', '.join(scene.get_object_names())}")
    except SceneLoadError as e:
        print(f"ERROR: Failed to load scene '{scene_name}': {e}")
        logger.app_logger.error(f"Failed to load scene '{scene_name}': {e}")
        logger.close()
        st.error(f"Failed to load scene '{scene_name}': {e}")
        st.stop()

    # Connect to PyBullet
    clid = p.connect(p.SHARED_MEMORY)
    if clid < 0:
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
    robot_controller.move_to_target([0.25, 0.25, 0.5], THRESHOLD_PRECISE)
    robot_controller.reset_orientation()
    for i in range(STABILIZATION_LOOP_STEPS):
        p.stepSimulation()

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
    robot_controller.reset_orientation()
    # Allow system to settle with correct orientation
    for i in range(STABILIZATION_LOOP_STEPS):
        p.stepSimulation()
    logger.app_logger.info(f"Stabilization loop completed: {STABILIZATION_LOOP_STEPS} steps")

    # Initialize Interactive LLM Controller
    logger.console_info("Initializing interactive LLM controller...")
    interactive_controller = InteractiveLLMController(
        robot_controller,
        object_manager,
        camera_manager,
        simulation_state,
        logger
    )
    logger.app_logger.info("Interactive LLM controller initialized")
    logger.console_info("Interactive mode ready!")

    return {
        "controller": interactive_controller,
        "object_manager": object_manager,
        "robot_controller": robot_controller,
        "logger": logger,
        "scene": scene
    }


def display_tool_results(tool_results):
    """Display tool execution results in the chat."""
    for tool_result in tool_results:
        tool_name = tool_result["tool_name"]
        tool_input = tool_result["tool_input"]
        result = tool_result["result"]

        # Create an expander for tool details
        with st.expander(f"🔧 Tool: {tool_name}", expanded=False):
            st.json({"input": tool_input, "result": result})


def main():
    """Main Streamlit app."""

    st.title("🤖 Robot Control Chat")
    st.markdown("Interact with the Franka Panda robot using natural language commands")

    # Initialize simulation (cached)
    with st.spinner("Initializing simulation..."):
        sim_components = initialize_simulation()

    controller = sim_components["controller"]
    object_manager = sim_components["object_manager"]
    robot_controller = sim_components["robot_controller"]
    scene = sim_components["scene"]

    # Initialize session state
    if "messages" not in st.session_state:
        st.session_state.messages = []
    if "conversation_history" not in st.session_state:
        st.session_state.conversation_history = []
    if "total_input_tokens" not in st.session_state:
        st.session_state.total_input_tokens = 0
    if "total_output_tokens" not in st.session_state:
        st.session_state.total_output_tokens = 0

    # Sidebar with scene information
    with st.sidebar:
        st.header("Scene Information")

        # Display scene metadata
        st.markdown(f"**Scene**: {scene.metadata.name}")
        st.caption(scene.metadata.description)
        st.divider()

        # Display objects
        st.subheader("Objects in Scene")
        objects_list = []
        for obj_name in object_manager.objects.keys():
            pos = object_manager.get_object_center_position(obj_name)
            objects_list.append(f"**{obj_name}**: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

        for obj_info in objects_list:
            st.markdown(obj_info)

        # Display gripper state
        st.subheader("Gripper State")
        gripper_pos = robot_controller.get_end_effector_position()
        gripper_state = "Open" if robot_controller.current_gripper_pos > 0.02 else "Closed"

        st.markdown(f"**Position**: [{gripper_pos[0]:.3f}, {gripper_pos[1]:.3f}, {gripper_pos[2]:.3f}]")
        st.markdown(f"**State**: {gripper_state}")

        # Display token usage
        st.divider()
        st.subheader("📊 Token Usage")
        st.markdown(f"**Input tokens**: {st.session_state.total_input_tokens:,}")
        st.markdown(f"**Output tokens**: {st.session_state.total_output_tokens:,}")
        st.markdown(f"**Total tokens**: {st.session_state.total_input_tokens + st.session_state.total_output_tokens:,}")

        # Add example commands
        st.divider()
        st.subheader("Example Commands")
        st.markdown("""
        - "What objects are in the scene?"
        - "Pick up the blue cube"
        - "Show me the scene"
        - "Move the gripper to [0.3, 0.4, 0.2]"
        - "Stack the red cube on the blue cube"
        - "Where is the green cube?"
        """)

        # Add refresh button
        if st.button("🔄 Refresh Scene Info"):
            st.rerun()

    # Display chat messages
    for message in st.session_state.messages:
        with st.chat_message(message["role"]):
            st.markdown(message["content"])

            # Display token usage for assistant messages
            if message["role"] == "assistant" and "token_usage" in message:
                usage = message["token_usage"]
                st.caption(f"🔢 Tokens: {usage['input_tokens']:,} in / {usage['output_tokens']:,} out")

            # Display panorama if present
            if "panorama" in message:
                st.image(message["panorama"], caption="Scene Panorama", use_container_width=True)

            # Display tool results if present
            if "tool_results" in message:
                display_tool_results(message["tool_results"])

    # Chat input
    if prompt := st.chat_input("Ask me anything about the robot or give me a command..."):
        # Add user message to chat
        st.session_state.messages.append({"role": "user", "content": prompt})

        with st.chat_message("user"):
            st.markdown(prompt)

        # Get response from LLM controller
        with st.chat_message("assistant"):
            with st.spinner("Thinking..."):
                try:
                    assistant_response, tool_results, panorama_base64, token_usage = controller.handle_message(
                        prompt,
                        st.session_state.conversation_history
                    )

                    # Update token counters
                    st.session_state.total_input_tokens += token_usage["input_tokens"]
                    st.session_state.total_output_tokens += token_usage["output_tokens"]

                    # Display assistant response
                    st.markdown(assistant_response)

                    # Display tool results if any
                    if tool_results:
                        display_tool_results(tool_results)

                    # Display panorama if captured
                    panorama_img = None
                    if panorama_base64:
                        img_data = base64.b64decode(panorama_base64)
                        panorama_img = Image.open(BytesIO(img_data))
                        st.image(panorama_img, caption="Scene Panorama", use_container_width=True)

                    # Add assistant message to chat
                    message_data = {
                        "role": "assistant",
                        "content": assistant_response,
                        "token_usage": token_usage
                    }
                    if tool_results:
                        message_data["tool_results"] = tool_results
                    if panorama_img:
                        message_data["panorama"] = panorama_img

                    st.session_state.messages.append(message_data)

                except Exception as e:
                    error_message = f"Error: {str(e)}"
                    st.error(error_message)
                    st.session_state.messages.append({
                        "role": "assistant",
                        "content": error_message
                    })

        # Rerun to update sidebar info
        st.rerun()


if __name__ == "__main__":
    main()
