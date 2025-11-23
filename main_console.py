"""
Console-based interactive mode for robot control.

This script provides a text-based chat interface for controlling the robot
simulation through natural language commands via Claude LLM. Unlike the
Streamlit version, this uses a direct GUI PyBullet connection (no shared memory).
"""

import pybullet as p
import os
import sys
import glob
import argparse
import base64
from io import BytesIO
from PIL import Image
from datetime import datetime

from src.config import *
from src.simulation_state import SimulationState
from src.object_manager import ObjectManager
from src.robot_controller import RobotController
from src.camera_manager import CameraManager
from src.interactive_llm_controller import InteractiveLLMController
from src.logger import SimulationLogger
from src.scene_loader import load_scene


class ConsoleSession:
    """Manages console session state (replaces Streamlit session state)."""

    def __init__(self):
        self.messages = []  # Chat display messages
        self.conversation_history = []  # API conversation history
        self.total_input_tokens = 0
        self.total_output_tokens = 0
        self.verbose = False  # Show tool details


def print_separator(char="=", length=70):
    """Print a separator line."""
    print(char * length)


def print_welcome():
    """Print welcome message and instructions."""
    print_separator("=")
    print("🤖 ROBOT CONTROL CONSOLE".center(70))
    print_separator("=")
    print("\nInteract with the Franka Panda robot using natural language commands.")
    print("Type '/help' for available commands, '/quit' to exit.\n")
    print_separator("-")


def print_help():
    """Print help information."""
    print("\n" + "="*70)
    print("AVAILABLE COMMANDS".center(70))
    print("="*70)
    print("\nSpecial Commands:")
    print("  /help      - Show this help message")
    print("  /status    - Display scene info, objects, gripper state, tokens")
    print("  /quit      - Exit the console")
    print("  /clear     - Clear conversation history")
    print("  /verbose   - Toggle verbose mode (show tool details)")
    print("\nExample Robot Commands:")
    print("  - What objects are in the scene?")
    print("  - Pick up the blue cube")
    print("  - Show me the scene")
    print("  - Move the gripper to [0.3, 0.4, 0.2]")
    print("  - Stack the red cube on the blue cube")
    print("  - Where is the green cube?")
    print("="*70 + "\n")


def print_status(session, object_manager, robot_controller):
    """Print current scene status."""
    print("\n" + "="*70)
    print("SCENE STATUS".center(70))
    print("="*70)

    # Objects
    print("\nObjects in Scene:")
    try:
        for obj_name in object_manager.objects.keys():
            try:
                pos = object_manager.get_object_center_position(obj_name)
                print(f"  - {obj_name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            except Exception as e:
                print(f"  - {obj_name}: Error getting position ({e})")
    except Exception as e:
        print(f"  Error displaying objects: {e}")

    # Gripper state
    print("\nGripper State:")
    try:
        gripper_pos = robot_controller.get_end_effector_position()
        gripper_state = "Open" if robot_controller.current_gripper_pos > 0.02 else "Closed"
        print(f"  Position: [{gripper_pos[0]:.3f}, {gripper_pos[1]:.3f}, {gripper_pos[2]:.3f}]")
        print(f"  State: {gripper_state}")
    except Exception as e:
        print(f"  Error displaying gripper state: {e}")

    # Token usage
    print("\nToken Usage:")
    total_tokens = session.total_input_tokens + session.total_output_tokens
    print(f"  Input:  {session.total_input_tokens:,} tokens")
    print(f"  Output: {session.total_output_tokens:,} tokens")
    print(f"  Total:  {total_tokens:,} tokens")

    # Verbose mode
    print(f"\nVerbose Mode: {'ON' if session.verbose else 'OFF'}")

    print("="*70 + "\n")


def display_tool_results(tool_results, verbose=False):
    """Display tool execution results."""
    if not tool_results:
        return

    print("\n  [Tools Used]")
    for i, tool_result in enumerate(tool_results, 1):
        tool_name = tool_result["tool_name"]
        tool_input = tool_result["tool_input"]
        result = tool_result["result"]

        if verbose:
            print(f"\n  Tool {i}: {tool_name}")
            print(f"    Input: {tool_input}")
            print(f"    Result: {result}")
        else:
            # Compact display
            print(f"    {i}. {tool_name}")


def save_panorama(panorama_base64, session_name="console"):
    """Save panorama image to file and return the path."""
    if not panorama_base64:
        return None

    try:
        img_data = base64.b64decode(panorama_base64)
        panorama_img = Image.open(BytesIO(img_data))

        # Create images folder if it doesn't exist
        os.makedirs(IMAGES_FOLDER, exist_ok=True)

        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"panorama_{session_name}_{timestamp}.jpg"
        filepath = os.path.join(IMAGES_FOLDER, filename)

        # Save image
        panorama_img.save(filepath)
        return filepath
    except Exception as e:
        print(f"  [Error saving panorama: {e}]")
        return None


def initialize_simulation(scene_name, logger):
    """Initialize PyBullet simulation and all components."""
    logger.console_info(f"Initializing console simulation with scene: {scene_name}...")

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
    logger.console_info("Simulation components initialized")

    # Clear images folder from previous runs
    images_folder = IMAGES_FOLDER
    if os.path.exists(images_folder):
        file_count = len(glob.glob(os.path.join(images_folder, '*')))
        for file in glob.glob(os.path.join(images_folder, '*')):
            os.remove(file)
        logger.app_logger.info(f"Cleared {file_count} files from {images_folder} folder")

    # Load scene
    logger.console_info(f"Loading scene: {scene_name}")
    scene = load_scene(scene_name)
    logger.console_info(f"Scene loaded: {scene.metadata.name}")

    # Load objects from scene
    logger.console_info("Loading objects...")
    for obj in scene.objects:
        if obj.type == 'cube':
            object_manager.load_cube(obj.name, obj.position, obj.color, obj.scale)
            logger.app_logger.info(f"Loaded cube: {obj.name} at {obj.position}")
        else:
            logger.app_logger.warning(f"Unknown object type: {obj.type}")

    logger.console_info(f"Loaded {len(object_manager.objects)} objects")

    # Stabilize robot
    logger.console_info("Stabilizing robot...")
    robot_controller.stabilize()
    logger.console_info("Robot stabilized")

    # Initialize Interactive LLM Controller
    logger.console_info("Initializing interactive LLM controller...")
    interactive_controller = InteractiveLLMController(
        robot_controller,
        object_manager,
        camera_manager,
        simulation_state,
        logger
    )
    logger.console_info("Interactive LLM controller initialized")
    logger.console_info(f"Registered objects: {list(object_manager.objects.keys())}")

    return {
        "controller": interactive_controller,
        "object_manager": object_manager,
        "robot_controller": robot_controller,
        "scene": scene,
        "logger": logger
    }


def main():
    """Main console application."""

    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Robot Control Console - Interactive chat interface')
    parser.add_argument('--scene', type=str, default='default',
                        help='Scene name to load (default: default)')
    args = parser.parse_args()

    # Initialize logger
    session_name = f"console_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    logger = SimulationLogger(log_dir=LOGS_FOLDER, session_name=session_name)

    # Initialize simulation
    try:
        print("\nInitializing simulation...")
        sim_components = initialize_simulation(args.scene, logger)
        controller = sim_components["controller"]
        object_manager = sim_components["object_manager"]
        robot_controller = sim_components["robot_controller"]
        scene = sim_components["scene"]

        print(f"Scene loaded: {scene.metadata.name}")
        print(f"Objects: {list(object_manager.objects.keys())}")

    except Exception as e:
        print(f"\nERROR: Failed to initialize simulation: {e}")
        logger.app_logger.error(f"Initialization failed: {e}", exc_info=True)
        return 1

    # Initialize session state
    session = ConsoleSession()

    # Print welcome message
    print_welcome()
    print(f"Scene: {scene.metadata.name}")
    print(f"Objects loaded: {len(object_manager.objects)}")
    print(f"Ready for commands!\n")
    print_separator("-")

    # Main chat loop
    try:
        while True:
            # Get user input
            try:
                user_input = input("\nYou: ").strip()
            except EOFError:
                print("\n\nExiting...")
                break

            if not user_input:
                continue

            # Handle special commands
            if user_input.startswith('/'):
                command = user_input.lower()

                if command == '/quit' or command == '/exit':
                    print("\nExiting console...")
                    break

                elif command == '/help':
                    print_help()
                    continue

                elif command == '/status':
                    print_status(session, object_manager, robot_controller)
                    continue

                elif command == '/clear':
                    session.conversation_history = []
                    session.messages = []
                    print("\n[Conversation history cleared]")
                    continue

                elif command == '/verbose':
                    session.verbose = not session.verbose
                    status = "ON" if session.verbose else "OFF"
                    print(f"\n[Verbose mode: {status}]")
                    continue

                else:
                    print(f"\n[Unknown command: {user_input}]")
                    print("[Type /help for available commands]")
                    continue

            # Add user message to session
            session.messages.append({"role": "user", "content": user_input})
            logger.log_interactive_message("user", user_input)

            # Get response from LLM controller
            print("\nAssistant: ", end="", flush=True)
            try:
                assistant_response, tool_results, panorama_base64, token_usage = controller.handle_message(
                    user_input,
                    session.conversation_history
                )

                # Update token counters
                session.total_input_tokens += token_usage["input_tokens"]
                session.total_output_tokens += token_usage["output_tokens"]

                # Display assistant response
                print(assistant_response)

                # Display tool results if any
                if tool_results:
                    display_tool_results(tool_results, verbose=session.verbose)

                # Save and display panorama if captured
                if panorama_base64:
                    filepath = save_panorama(panorama_base64, session_name)
                    if filepath:
                        print(f"\n  [Panorama saved: {filepath}]")

                # Display token usage for this turn
                total_turn_tokens = token_usage["input_tokens"] + token_usage["output_tokens"]
                print(f"\n  [Tokens: {token_usage['input_tokens']:,} in / {token_usage['output_tokens']:,} out | Total this turn: {total_turn_tokens:,}]")

                # Add assistant message to session
                message_data = {
                    "role": "assistant",
                    "content": assistant_response,
                    "token_usage": token_usage
                }
                if tool_results:
                    message_data["tool_results"] = tool_results

                session.messages.append(message_data)
                logger.log_interactive_message("assistant", assistant_response)

            except KeyboardInterrupt:
                print("\n\n[Interrupted]")
                raise
            except Exception as e:
                error_message = f"Error: {str(e)}"
                print(f"\n{error_message}")
                logger.app_logger.error(f"Error handling message: {e}", exc_info=True)
                session.messages.append({
                    "role": "assistant",
                    "content": error_message
                })

            print_separator("-")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting...")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        logger.app_logger.error(f"Unexpected error in main loop: {e}", exc_info=True)
        return 1
    finally:
        # Cleanup
        print("\nCleaning up...")
        try:
            p.disconnect()
            print("PyBullet disconnected")
        except:
            pass

        # Print final stats
        print("\n" + "="*70)
        print("SESSION SUMMARY".center(70))
        print("="*70)
        print(f"Messages exchanged: {len(session.messages)}")
        print(f"Total input tokens: {session.total_input_tokens:,}")
        print(f"Total output tokens: {session.total_output_tokens:,}")
        print(f"Total tokens: {session.total_input_tokens + session.total_output_tokens:,}")
        print("="*70)
        print("\nGoodbye!\n")
        logger.console_info("Console session ended")

    return 0


if __name__ == "__main__":
    sys.exit(main())
