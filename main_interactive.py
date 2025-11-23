"""
Interactive mode for robot control using Streamlit chat interface.

This script launches a Streamlit web app that allows users to interact with
the robot simulation through natural language commands via Claude LLM.
"""

import pybullet as p
import os
import glob
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




@st.cache_resource
def initialize_simulation():
    """Initialize PyBullet simulation and all components (cached).

    Connects to PyBullet in GUI mode and loads the simulation environment.
    """

    # Initialize logger
    logger = SimulationLogger(log_dir=LOGS_FOLDER, session_name=LOG_SESSION_NAME)
    logger.console_info("Initializing interactive simulation...")

    # Connect to PyBullet GUI
    logger.console_info("Starting PyBullet GUI...")
    armId = RobotController.initialize_pybullet(logger=logger)
    endEffectorIndex = END_EFFECTOR_INDEX

    logger.console_info("PyBullet initialized in GUI mode")

    # Initialize simulation components
    logger.console_info("Initializing simulation components...")
    simulation_state = SimulationState()
    object_manager = ObjectManager(logger)
    camera_manager = CameraManager(logger)
    robot_controller = RobotController(armId, endEffectorIndex, simulation_state, object_manager, camera_manager, logger)
    logger.app_logger.info("Simulation components initialized")
    logger.console_info("Simulation components initialized")

    # Clear images folder from previous runs
    images_folder = IMAGES_FOLDER
    if os.path.exists(images_folder):
        file_count = len(glob.glob(os.path.join(images_folder, '*')))
        for file in glob.glob(os.path.join(images_folder, '*')):
            os.remove(file)
        logger.app_logger.info(f"Cleared {file_count} files from {images_folder} folder")

    # Discover and register objects already loaded by the visual server
    logger.console_info("Discovering objects from visual server...")
    num_bodies = p.getNumBodies()
    logger.console_info(f"Found {num_bodies} bodies in simulation")

    for i in range(num_bodies):
        body_info = p.getBodyInfo(i)
        body_name = body_info[0].decode('utf-8')
        logger.console_info(f"Body {i}: {body_name}")

        # Register objects (skip plane and robot)
        if body_name not in ['plane.urdf', 'panda.urdf', ''] and 'plane' not in body_name.lower() and 'panda' not in body_name.lower():
            # Extract object name from the body name (usually in format: path/name.urdf or just name)
            obj_name = body_name.replace('.urdf', '').split('/')[-1]
            object_manager.objects[obj_name] = i
            logger.app_logger.info(f"Registered object: {obj_name} (ID: {i})")

    logger.console_info(f"Registered {len(object_manager.objects)} objects from visual server")

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
    logger.console_info("Interactive LLM controller initialized")

    # Log registered objects
    logger.console_info(f"Registered objects: {list(object_manager.objects.keys())}")
    logger.console_info("Interactive mode ready!")

    return {
        "controller": interactive_controller,
        "object_manager": object_manager,
        "robot_controller": robot_controller,
        "logger": logger
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
    try:
        with st.spinner("Initializing simulation..."):
            sim_components = initialize_simulation()
    except Exception as e:
        st.error(f"Failed to initialize simulation: {e}")
        st.stop()

    controller = sim_components["controller"]
    object_manager = sim_components["object_manager"]
    robot_controller = sim_components["robot_controller"]

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

        # Display objects
        st.subheader("Objects in Scene")
        try:
            objects_list = []
            for obj_name in object_manager.objects.keys():
                try:
                    pos = object_manager.get_object_center_position(obj_name)
                    objects_list.append(f"**{obj_name}**: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
                except Exception as e:
                    objects_list.append(f"**{obj_name}**: Error getting position ({e})")

            for obj_info in objects_list:
                st.markdown(obj_info)
        except Exception as e:
            st.error(f"Error displaying objects: {e}")

        # Display gripper state
        st.subheader("Gripper State")
        try:
            gripper_pos = robot_controller.get_end_effector_position()
            gripper_state = "Open" if robot_controller.current_gripper_pos > 0.02 else "Closed"

            st.markdown(f"**Position**: [{gripper_pos[0]:.3f}, {gripper_pos[1]:.3f}, {gripper_pos[2]:.3f}]")
            st.markdown(f"**State**: {gripper_state}")
        except Exception as e:
            st.error(f"Error displaying gripper state: {e}")

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
