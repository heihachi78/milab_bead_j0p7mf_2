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
import time
import select
from io import BytesIO
from PIL import Image
from datetime import datetime

from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.markdown import Markdown
from rich.rule import Rule
from rich.status import Status
from rich.text import Text

from src.config import *
from src.simulation_state import SimulationState
from src.object_manager import ObjectManager
from src.robot_controller import RobotController
from src.camera_manager import CameraManager
from src.interactive_llm_controller import InteractiveLLMController
from src.logger import SimulationLogger
from src.scene_loader import load_scene

# Initialize Rich console
console = Console()


class ConsoleSession:
    """Manages console session state (replaces Streamlit session state)."""

    def __init__(self):
        self.messages = []  # Chat display messages
        self.conversation_history = []  # API conversation history
        self.total_input_tokens = 0
        self.total_output_tokens = 0
        self.verbose = False  # Show tool details


def print_welcome():
    """Print welcome message and instructions."""
    welcome_text = Text()
    welcome_text.append("🤖 ROBOT CONTROL CONSOLE\n\n", style="bold cyan")
    welcome_text.append("Interact with the Franka Panda robot using natural language commands.\n", style="white")
    welcome_text.append("Type ", style="dim")
    welcome_text.append("/help", style="bold yellow")
    welcome_text.append(" for available commands, ", style="dim")
    welcome_text.append("/quit", style="bold yellow")
    welcome_text.append(" to exit.", style="dim")

    console.print(Panel(welcome_text, border_style="cyan", padding=(1, 2)))
    console.print()


def print_help():
    """Print help information."""
    console.print()

    # Special commands table
    special_table = Table(title="Special Commands", border_style="yellow", show_header=True, header_style="bold yellow")
    special_table.add_column("Command", style="cyan", width=15)
    special_table.add_column("Description", style="white")

    special_table.add_row("/help", "Show this help message")
    special_table.add_row("/status", "Display scene info, objects, gripper state, tokens")
    special_table.add_row("/quit", "Exit the console")
    special_table.add_row("/clear", "Clear conversation history")
    special_table.add_row("/verbose", "Toggle verbose mode (show tool details)")

    console.print(special_table)
    console.print()

    # Example commands panel
    examples = Text()
    examples.append("Example Robot Commands:\n\n", style="bold green")
    examples.append("• What objects are in the scene?\n", style="white")
    examples.append("• Pick up the blue cube\n", style="white")
    examples.append("• Show me the scene\n", style="white")
    examples.append("• Move the gripper to [0.3, 0.4, 0.2]\n", style="white")
    examples.append("• Stack the red cube on the blue cube\n", style="white")
    examples.append("• Where is the green cube?", style="white")

    console.print(Panel(examples, border_style="green", padding=(1, 2)))
    console.print()


def print_status(session, object_manager, robot_controller):
    """Print current scene status."""
    console.print()

    # Objects table
    objects_table = Table(title="Objects in Scene", border_style="blue", show_header=True, header_style="bold blue")
    objects_table.add_column("Object Name", style="cyan")
    objects_table.add_column("Position [x, y, z]", style="white", justify="right")

    try:
        for obj_name in object_manager.objects.keys():
            try:
                pos = object_manager.get_object_center_position(obj_name)
                objects_table.add_row(obj_name, f"[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            except Exception as e:
                objects_table.add_row(obj_name, f"[red]Error: {e}[/red]")
    except Exception as e:
        console.print(f"[red]Error displaying objects: {e}[/red]")

    console.print(objects_table)
    console.print()

    # Gripper state panel
    try:
        gripper_pos = robot_controller.get_end_effector_position()
        gripper_state = "Open" if robot_controller.current_gripper_pos > 0.02 else "Closed"
        gripper_emoji = "🟢" if robot_controller.current_gripper_pos > 0.02 else "🔴"

        gripper_info = Text()
        gripper_info.append(f"{gripper_emoji} State: ", style="bold")
        gripper_info.append(f"{gripper_state}\n", style="green" if gripper_state == "Open" else "red")
        gripper_info.append("Position: ", style="bold")
        gripper_info.append(f"[{gripper_pos[0]:.3f}, {gripper_pos[1]:.3f}, {gripper_pos[2]:.3f}]", style="white")

        console.print(Panel(gripper_info, title="Gripper State", border_style="magenta", padding=(1, 2)))
    except Exception as e:
        console.print(f"[red]Error displaying gripper state: {e}[/red]")

    console.print()

    # Token usage table
    token_table = Table(border_style="yellow", show_header=True, header_style="bold yellow")
    token_table.add_column("Type", style="cyan")
    token_table.add_column("Tokens", justify="right", style="white")

    total_tokens = session.total_input_tokens + session.total_output_tokens
    token_table.add_row("Input", f"{session.total_input_tokens:,}")
    token_table.add_row("Output", f"{session.total_output_tokens:,}")
    token_table.add_row("Total", f"{total_tokens:,}", style="bold green")

    console.print(token_table)
    console.print()

    # Verbose mode
    verbose_status = "🔊 ON" if session.verbose else "🔇 OFF"
    verbose_color = "green" if session.verbose else "dim"
    console.print(f"Verbose Mode: [{verbose_color}]{verbose_status}[/{verbose_color}]")
    console.print()


def display_tool_results(tool_results, verbose=False):
    """Display tool execution results."""
    if not tool_results:
        return

    if verbose:
        # Detailed view with table
        tools_table = Table(title="🔧 Tools Used", border_style="yellow", show_header=True, header_style="bold yellow")
        tools_table.add_column("#", style="dim", width=3)
        tools_table.add_column("Tool", style="cyan")
        tools_table.add_column("Input", style="white")
        tools_table.add_column("Result", style="green")

        for i, tool_result in enumerate(tool_results, 1):
            tool_name = tool_result["tool_name"]
            tool_input = str(tool_result["tool_input"])
            result = str(tool_result["result"])

            # Truncate long results
            if len(result) > 50:
                result = result[:50] + "..."

            tools_table.add_row(str(i), tool_name, tool_input, result)

        console.print(tools_table)
    else:
        # Compact view
        tool_names = [f"{i}. {tr['tool_name']}" for i, tr in enumerate(tool_results, 1)]
        tools_text = Text("🔧 Tools: ", style="dim yellow")
        tools_text.append(" • ".join(tool_names), style="dim cyan")
        console.print(tools_text)


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
        console.print(f"[red]Error saving panorama: {e}[/red]")
        return None


def get_input_with_simulation():
    """
    Get user input while continuously stepping the simulation.
    Uses non-blocking select() to check for input while running physics.

    The terminal is in line-buffered mode, so we step the simulation until
    the user presses Enter, then read the complete line.

    Returns:
        str: User input string, or raises EOFError if interrupted
    """
    console.print("\n[bold blue]You:[/bold blue] ", end="")

    while True:
        # Check if input is available (non-blocking)
        # select() with timeout=0 returns immediately
        readable, _, _ = select.select([sys.stdin], [], [], 0)

        if readable:
            # Input is available (user pressed Enter), read the complete line
            line = sys.stdin.readline()
            if line == '':
                # EOF (Ctrl+D)
                raise EOFError()
            # Remove trailing newline and return
            return line.rstrip('\n')
        else:
            # No input available, step simulation
            p.stepSimulation()
            time.sleep(TIME_STEP)  # Maintain consistent physics rate


def initialize_simulation(scene_name, logger):
    """Initialize PyBullet simulation and all components."""
    with console.status(f"[cyan]Initializing console simulation with scene: {scene_name}...", spinner="dots"):
        # Connect to PyBullet GUI
        armId = RobotController.initialize_pybullet(logger=logger)
        endEffectorIndex = END_EFFECTOR_INDEX

    console.print("[green]✓[/green] PyBullet initialized in GUI mode")

    # Initialize simulation components
    with console.status("[cyan]Initializing simulation components...", spinner="dots"):
        simulation_state = SimulationState()
        object_manager = ObjectManager(logger)
        camera_manager = CameraManager(logger)
        robot_controller = RobotController(armId, endEffectorIndex, simulation_state, object_manager, camera_manager, logger)

    console.print("[green]✓[/green] Simulation components initialized")

    # Clear images folder from previous runs
    images_folder = IMAGES_FOLDER
    if os.path.exists(images_folder):
        file_count = len(glob.glob(os.path.join(images_folder, '*')))
        for file in glob.glob(os.path.join(images_folder, '*')):
            os.remove(file)
        logger.app_logger.info(f"Cleared {file_count} files from {images_folder} folder")

    # Load scene
    with console.status(f"[cyan]Loading scene: {scene_name}...", spinner="dots"):
        scene = load_scene(scene_name)

    console.print(f"[green]✓[/green] Scene loaded: [bold]{scene.metadata.name}[/bold]")

    # Load objects from scene
    with console.status("[cyan]Loading objects...", spinner="dots"):
        for obj in scene.objects:
            if obj.type == 'cube':
                object_manager.load_cube(obj.name, obj.position, obj.color, obj.scale)
                logger.app_logger.info(f"Loaded cube: {obj.name} at {obj.position}")
            else:
                logger.app_logger.warning(f"Unknown object type: {obj.type}")

    console.print(f"[green]✓[/green] Loaded {len(object_manager.objects)} objects")

    # Stabilization sequence
    with console.status("[cyan]🤖 Stabilizing robot...", spinner="dots"):
        # First stabilize robot at rest pose with motors active
        robot_controller.stabilize()
        # Reset gripper orientation (move_to_target runs simulation internally)
        robot_controller.move_to_target([0.25, 0.25, 0.5], THRESHOLD_PRECISE)
        robot_controller.reset_orientation()
        # Allow system to settle with correct orientation
        for i in range(STABILIZATION_LOOP_STEPS):
            p.stepSimulation()

    console.print("[green]✓[/green] Robot stabilized")

    # Capture initial panorama after stabilization
    with console.status("[cyan]📸 Capturing initial panorama...", spinner="dots"):
        camera_manager.capture_and_save_panorama("initial_stabilized")

    console.print("[green]✓[/green] Initial panorama captured")

    # Initialize Interactive LLM Controller
    with console.status("[cyan]Initializing interactive LLM controller...", spinner="dots"):
        interactive_controller = InteractiveLLMController(
            robot_controller,
            object_manager,
            camera_manager,
            simulation_state,
            logger
        )

    console.print("[green]✓[/green] Interactive LLM controller initialized")
    logger.app_logger.info(f"Registered objects: {list(object_manager.objects.keys())}")

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
        console.print()
        sim_components = initialize_simulation(args.scene, logger)
        controller = sim_components["controller"]
        object_manager = sim_components["object_manager"]
        robot_controller = sim_components["robot_controller"]
        scene = sim_components["scene"]
        console.print()

    except Exception as e:
        console.print(f"\n[bold red]ERROR:[/bold red] Failed to initialize simulation: {e}")
        logger.app_logger.error(f"Initialization failed: {e}", exc_info=True)
        return 1

    # Initialize session state
    session = ConsoleSession()

    # Print welcome message
    print_welcome()

    # Scene info panel
    scene_info = Text()
    scene_info.append("Scene: ", style="bold")
    scene_info.append(f"{scene.metadata.name}\n", style="cyan")
    scene_info.append("Objects: ", style="bold")
    scene_info.append(f"{len(object_manager.objects)}", style="green")
    console.print(Panel(scene_info, title="🎬 Simulation Ready", border_style="green", padding=(0, 2)))
    console.print()

    # Main chat loop
    try:
        while True:
            # Get user input with continuous simulation
            try:
                user_input = get_input_with_simulation().strip()
            except EOFError:
                console.print("\n\n[yellow]Exiting...[/yellow]")
                break

            if not user_input:
                continue

            # Handle special commands
            if user_input.startswith('/'):
                command = user_input.lower()

                if command == '/quit' or command == '/exit':
                    console.print("\n[yellow]Exiting console...[/yellow]")
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
                    console.print("\n[green]✓ Conversation history cleared[/green]")
                    continue

                elif command == '/verbose':
                    session.verbose = not session.verbose
                    status = "🔊 ON" if session.verbose else "🔇 OFF"
                    color = "green" if session.verbose else "yellow"
                    console.print(f"\n[{color}]Verbose mode: {status}[/{color}]")
                    continue

                else:
                    console.print(f"\n[red]Unknown command: {user_input}[/red]")
                    console.print("[dim]Type /help for available commands[/dim]")
                    continue

            # Add user message to session
            session.messages.append({"role": "user", "content": user_input})
            logger.log_interactive_message("user", user_input)

            # Get response from LLM controller with status indicator
            console.print()
            try:
                with console.status("[bold cyan]🤖 Thinking...", spinner="dots"):
                    assistant_response, tool_results, panorama_base64, token_usage = controller.handle_message(
                        user_input,
                        session.conversation_history
                    )

                # Update token counters
                session.total_input_tokens += token_usage["input_tokens"]
                session.total_output_tokens += token_usage["output_tokens"]

                # Display assistant response in a panel
                response_panel = Panel(
                    Markdown(assistant_response),
                    title="🤖 Assistant",
                    border_style="green",
                    padding=(1, 2)
                )
                console.print(response_panel)

                # Display tool results if any
                if tool_results:
                    display_tool_results(tool_results, verbose=session.verbose)

                # Save and display panorama if captured
                if panorama_base64:
                    filepath = save_panorama(panorama_base64, session_name)
                    if filepath:
                        console.print(f"[dim]📸 Panorama saved: {filepath}[/dim]")

                # Display token usage for this turn
                total_turn_tokens = token_usage["input_tokens"] + token_usage["output_tokens"]
                token_info = Text()
                token_info.append("💰 Tokens: ", style="dim")
                token_info.append(f"{token_usage['input_tokens']:,} in", style="cyan")
                token_info.append(" / ", style="dim")
                token_info.append(f"{token_usage['output_tokens']:,} out", style="magenta")
                token_info.append(" | Total: ", style="dim")
                token_info.append(f"{total_turn_tokens:,}", style="bold green")
                console.print(token_info)

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
                console.print("\n\n[yellow]⚠ Interrupted[/yellow]")
                raise
            except Exception as e:
                error_message = f"Error: {str(e)}"
                console.print(Panel(error_message, title="❌ Error", border_style="red", padding=(1, 2)))
                logger.app_logger.error(f"Error handling message: {e}", exc_info=True)
                session.messages.append({
                    "role": "assistant",
                    "content": error_message
                })

            console.rule(style="dim")

    except KeyboardInterrupt:
        console.print("\n\n[yellow]Interrupted by user. Exiting...[/yellow]")
    except Exception as e:
        console.print(f"\n\n[bold red]Unexpected error: {e}[/bold red]")
        logger.app_logger.error(f"Unexpected error in main loop: {e}", exc_info=True)
        return 1
    finally:
        # Cleanup
        console.print("\n[cyan]Cleaning up...[/cyan]")
        try:
            p.disconnect()
            console.print("[green]✓ PyBullet disconnected[/green]")
        except:
            pass

        # Print final stats
        console.print()
        summary_table = Table(title="📊 Session Summary", border_style="cyan", show_header=True, header_style="bold cyan")
        summary_table.add_column("Metric", style="white")
        summary_table.add_column("Value", justify="right", style="green")

        summary_table.add_row("Messages exchanged", str(len(session.messages)))
        summary_table.add_row("Total input tokens", f"{session.total_input_tokens:,}")
        summary_table.add_row("Total output tokens", f"{session.total_output_tokens:,}")
        summary_table.add_row("Total tokens", f"{session.total_input_tokens + session.total_output_tokens:,}")

        console.print(summary_table)
        console.print("\n[bold cyan]👋 Goodbye![/bold cyan]\n")
        logger.console_info("Console session ended")

    return 0


if __name__ == "__main__":
    sys.exit(main())
