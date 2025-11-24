import os
import json
import base64
from pathlib import Path
from typing import Dict, Any, List, Optional
from dotenv import load_dotenv
from io import BytesIO

import anthropic
from PIL import Image

from . import config


class InteractiveLLMController:
    """Interactive LLM controller for conversational robot control using Claude's native tool calling."""

    def __init__(self, robot_controller, object_manager, camera_manager, simulation_state, logger=None):
        """
        Initialize the interactive LLM controller.

        Args:
            robot_controller: RobotController instance for robot operations
            object_manager: ObjectManager instance for querying object positions
            camera_manager: CameraManager instance for panorama capture
            simulation_state: SimulationState instance for state tracking
            logger: SimulationLogger instance for logging (optional)
        """
        self.robot_controller = robot_controller
        self.object_manager = object_manager
        self.camera_manager = camera_manager
        self.simulation_state = simulation_state
        self.logger = logger

        # Load environment variables
        load_dotenv()
        self.api_key = os.getenv('ANTHROPIC_API_KEY')
        if not self.api_key:
            raise ValueError("ANTHROPIC_API_KEY not found in .env file")

        # Get model from environment
        self.model = os.getenv('ANTHROPIC_MODEL', 'claude-sonnet-4-5-20250929')

        # Initialize Anthropic client
        self.client = anthropic.Anthropic(api_key=self.api_key)
        self.max_tokens = config.INTERACTIVE_MAX_TOKENS if hasattr(config, 'INTERACTIVE_MAX_TOKENS') else 4096

        # Load system prompt
        self.system_prompt = self._load_system_prompt()

        # Log system prompt at initialization
        if self.logger:
            self.logger.interactive_logger.info("=" * 80)
            self.logger.interactive_logger.info("INTERACTIVE MODE INITIALIZED")
            self.logger.interactive_logger.info("-" * 80)
            self.logger.interactive_logger.info("SYSTEM PROMPT:")
            self.logger.interactive_logger.info(self.system_prompt)
            self.logger.interactive_logger.info("=" * 80)
            self.logger.interactive_logger.info("")  # Empty line for readability

        # Conversation history (managed externally, passed to handle_message)
        # Format: [{"role": "user", "content": "..."}, {"role": "assistant", "content": [...]}]

    def _load_system_prompt(self) -> str:
        """Load system prompt for interactive mode."""
        prompt_file = config.INTERACTIVE_SYSTEM_PROMPT_FILE if hasattr(config, 'INTERACTIVE_SYSTEM_PROMPT_FILE') else 'interactive_system_prompt.txt'
        filepath = Path(__file__).parent.parent / 'prompts' / prompt_file

        if not filepath.exists():
            # Return default system prompt if file doesn't exist
            return self._get_default_system_prompt()

        with open(filepath, 'r') as f:
            return f.read()

    def _get_default_system_prompt(self) -> str:
        """Return default system prompt if file doesn't exist."""
        return """You are a helpful robotics assistant controlling a Franka Panda robotic arm in a PyBullet simulation."""

    def get_tool_definitions(self) -> List[Dict[str, Any]]:
        """
        Define all available tools for the LLM to use.

        Returns:
            List of tool definition dictionaries
        """
        return [
            {
                "name": "get_gripper_position",
                "description": "Get the current position of the robot's end effector (gripper) in 3D space.",
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            },
            {
                "name": "get_gripper_state",
                "description": "Get the current state of the gripper (open or closed).",
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            },
            {
                "name": "get_object_position",
                "description": "Get the current position of a specific object in the scene.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "object_name": {
                            "type": "string",
                            "description": "Name of the object (e.g., 'blue_cube', 'red_cube')"
                        }
                    },
                    "required": ["object_name"]
                }
            },
            {
                "name": "get_all_objects",
                "description": "Get a list of all objects in the scene with their positions and dimensions.",
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            },
            {
                "name": "get_panorama",
                "description": "Capture a panorama image of the current scene from multiple camera angles. Use this when you need visual understanding of the scene layout.",
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            },
            {
                "name": "move_gripper",
                "description": "Move the gripper to a specific position using IK-based movement. This is standard movement (not smooth linear interpolation). Use this when you need to move large distances and precision is not too important.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "x": {
                            "type": "number",
                            "description": "X coordinate in meters"
                        },
                        "y": {
                            "type": "number",
                            "description": "Y coordinate in meters"
                        },
                        "z": {
                            "type": "number",
                            "description": "Z coordinate in meters"
                        }
                    },
                    "required": ["x", "y", "z"]
                }
            },
            {
                "name": "move_gripper_smooth",
                "description": "Move the gripper to a specific position using smooth linear interpolation at constant velocity. Use this for precise, gentle movements, but only for short distances.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "x": {
                            "type": "number",
                            "description": "X coordinate in meters"
                        },
                        "y": {
                            "type": "number",
                            "description": "Y coordinate in meters"
                        },
                        "z": {
                            "type": "number",
                            "description": "Z coordinate in meters"
                        }
                    },
                    "required": ["x", "y", "z"]
                }
            },
            {
                "name": "open_gripper",
                "description": "Open the gripper to release an object or prepare to grasp.",
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            },
            {
                "name": "close_gripper",
                "description": "Close the gripper to grasp an object.",
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            },
            {
                "name": "rotate_gripper_90",
                "description": "Use this to avoid hitting close objects with the gripper's wide part along the Y axis during pick up and place operations.",
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            },
            {
                "name": "reset_gripper_orientation",
                "description": "Use this to avoid hitting close objects with the gripper's wide part along the X axis during pick up and place operations.",
                "input_schema": {
                    "type": "object",
                    "properties": {},
                    "required": []
                }
            },
            {
                "name": "pick_up_object",
                "description": "Pick up an object from above (approach, open gripper, grasp, lift). Does NOT avoid obstacles - target must be clear from above.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "object_name": {
                            "type": "string",
                            "description": "Name of the object to pick up (e.g., 'blue_cube')"
                        }
                    },
                    "required": ["object_name"]
                }
            },
            {
                "name": "place_object",
                "description": "Place the held object at specific coordinates (approach, descend, release, retract).  Does NOT avoid obstacles.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "x": {
                            "type": "number",
                            "description": "X coordinate in meters"
                        },
                        "y": {
                            "type": "number",
                            "description": "Y coordinate in meters"
                        },
                        "z": {
                            "type": "number",
                            "description": "Z coordinate in meters (ground level ≈ 0.0)"
                        }
                    },
                    "required": ["x", "y", "z"]
                }
            },
            {
                "name": "place_on_object",
                "description": "Place the held object on top of another object (approach, descend, release, retract). Does NOT avoid obstacles.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "target_object": {
                            "type": "string",
                            "description": "Name of the object to place on (e.g., 'red_cube')"
                        }
                    },
                    "required": ["target_object"]
                }
            }
        ]

    def execute_tool(self, tool_name: str, tool_input: Dict[str, Any]) -> Dict[str, Any]:
        """
        Execute a tool and return the result.

        Args:
            tool_name: Name of the tool to execute
            tool_input: Input parameters for the tool

        Returns:
            Dictionary with tool execution result
        """
        if self.logger:
            self.logger.log_tool_call(tool_name, tool_input)

        try:
            result = self._execute_tool_impl(tool_name, tool_input)

            if self.logger:
                self.logger.log_tool_result(tool_name, result)

            return result

        except Exception as e:
            error_result = {
                "success": False,
                "error": str(e),
                "message": f"Error executing {tool_name}: {str(e)}"
            }

            if self.logger:
                self.logger.log_tool_result(tool_name, error_result)

            return error_result

    def _execute_tool_impl(self, tool_name: str, tool_input: Dict[str, Any]) -> Dict[str, Any]:
        """Internal implementation of tool execution."""

        if tool_name == "get_gripper_position":
            position = self.robot_controller.get_end_effector_position()
            return {
                "success": True,
                "position": position,
                "message": f"Gripper position: [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}] meters"
            }

        elif tool_name == "get_gripper_state":
            state = "open" if self.robot_controller.current_gripper_pos > 0.02 else "closed"
            return {
                "success": True,
                "state": state,
                "position": self.robot_controller.current_gripper_pos,
                "message": f"Gripper is {state} (position: {self.robot_controller.current_gripper_pos:.4f})"
            }

        elif tool_name == "get_object_position":
            object_name = tool_input["object_name"]
            position = self.object_manager.get_object_center_position(object_name)
            return {
                "success": True,
                "object": object_name,
                "position": position,
                "message": f"{object_name} position: [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}] meters"
            }

        elif tool_name == "get_all_objects":
            objects_info = []
            for obj_name in self.object_manager.objects.keys():
                position = self.object_manager.get_object_center_position(obj_name)
                obj_id = self.object_manager.get_object_id(obj_name)
                dimensions = self.object_manager.get_object_dimensions(obj_id)
                objects_info.append({
                    "name": obj_name,
                    "position": position,
                    "dimensions": dimensions
                })

            message = "Objects in scene:\n"
            for obj in objects_info:
                message += f"- {obj['name']}: position [{obj['position'][0]:.3f}, {obj['position'][1]:.3f}, {obj['position'][2]:.3f}], "
                message += f"size [{obj['dimensions'][0]:.3f}, {obj['dimensions'][1]:.3f}, {obj['dimensions'][2]:.3f}] meters\n"

            return {
                "success": True,
                "objects": objects_info,
                "count": len(objects_info),
                "message": message.strip()
            }

        elif tool_name == "get_panorama":
            images = self.camera_manager.capture_multi_camera()
            panorama = self.camera_manager.create_panorama(images)

            # Convert PIL image to base64 using JPEG compression to reduce token usage
            buffered = BytesIO()
            format_type = config.PANORAMA_FORMAT if hasattr(config, 'PANORAMA_FORMAT') else 'JPEG'
            if format_type == 'JPEG':
                quality = config.PANORAMA_QUALITY if hasattr(config, 'PANORAMA_QUALITY') else 75
                panorama.save(buffered, format="JPEG", quality=quality, optimize=True)
            else:
                panorama.save(buffered, format="PNG")
            img_str = base64.b64encode(buffered.getvalue()).decode()

            return {
                "success": True,
                "panorama_base64": img_str,
                "message": f"Panorama captured successfully (5-viewpoint composite image, {format_type} format)"
            }

        elif tool_name == "move_gripper":
            position = [tool_input["x"], tool_input["y"], tool_input["z"]]
            threshold = config.THRESHOLD_CLOSE_TARGET if hasattr(config, 'THRESHOLD_CLOSE_TARGET') else 0.01
            final_pos = self.robot_controller.move_to_target(position, threshold)
            return {
                "success": True,
                "target_position": position,
                "final_position": list(final_pos),
                "message": f"Moved gripper to [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}]"
            }

        elif tool_name == "move_gripper_smooth":
            position = [tool_input["x"], tool_input["y"], tool_input["z"]]
            threshold = config.THRESHOLD_PRECISE if hasattr(config, 'THRESHOLD_PRECISE') else 0.001
            final_pos = self.robot_controller.move_to_target_smooth(position, threshold)
            return {
                "success": True,
                "target_position": position,
                "final_position": list(final_pos),
                "message": f"Smoothly moved gripper to [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}]"
            }

        elif tool_name == "open_gripper":
            self.robot_controller.open_gripper()
            return {
                "success": True,
                "message": "Gripper opened"
            }

        elif tool_name == "close_gripper":
            self.robot_controller.close_gripper()
            return {
                "success": True,
                "message": "Gripper closed"
            }

        elif tool_name == "rotate_gripper_90":
            self.robot_controller.rotate_orientation_90()
            return {
                "success": True,
                "message": "Gripper rotated 90 degrees (orientation: [0.0, -π, -π/2])"
            }

        elif tool_name == "reset_gripper_orientation":
            self.robot_controller.reset_orientation()
            return {
                "success": True,
                "message": "Gripper orientation reset to default (orientation: [0.0, -π, 0.0])"
            }

        elif tool_name == "pick_up_object":
            object_name = tool_input["object_name"]
            final_pos = self.robot_controller.pick_up(object_name)
            return {
                "success": True,
                "object": object_name,
                "final_position": list(final_pos),
                "message": f"Successfully picked up {object_name}"
            }

        elif tool_name == "place_object":
            position = [tool_input["x"], tool_input["y"], tool_input["z"]]
            final_pos = self.robot_controller.place(position)
            return {
                "success": True,
                "position": position,
                "final_position": list(final_pos),
                "message": f"Successfully placed object at [{position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}]"
            }

        elif tool_name == "place_on_object":
            target_object = tool_input["target_object"]
            final_pos = self.robot_controller.place_on(target_object)
            return {
                "success": True,
                "target_object": target_object,
                "final_position": list(final_pos),
                "message": f"Successfully placed object on {target_object}"
            }

        else:
            return {
                "success": False,
                "error": f"Unknown tool: {tool_name}",
                "message": f"Tool '{tool_name}' is not recognized"
            }

    def handle_message(self, user_message: str, conversation_history: List[Dict[str, Any]]) -> tuple[str, List[Dict[str, Any]], Optional[str], Dict[str, int]]:
        """
        Handle a user message and return the assistant's response.

        Args:
            user_message: The user's message text
            conversation_history: List of previous messages in the conversation

        Returns:
            Tuple of (assistant_response_text, tool_results_list, panorama_base64 or None, token_usage)
            - assistant_response_text: The text response from the assistant
            - tool_results_list: List of tool execution results with details
            - panorama_base64: Base64 encoded panorama if get_panorama was called, else None
            - token_usage: Dictionary with 'input_tokens' and 'output_tokens' counts
        """
        # Add user message to history
        conversation_history.append({
            "role": "user",
            "content": user_message
        })

        if self.logger:
            self.logger.log_interactive_message("user", user_message)

        # Prepare system prompt with caching
        system_with_cache = [
            {
                "type": "text",
                "text": self.system_prompt,
                "cache_control": {"type": "ephemeral"}
            }
        ]

        # Prepare tools with caching (mark last tool to cache all)
        tools_with_cache = self.get_tool_definitions()
        if tools_with_cache:
            tools_with_cache[-1]["cache_control"] = {"type": "ephemeral"}

        # Log API request (before making the call)
        if self.logger:
            self.logger.log_api_request(
                model=self.model,
                max_tokens=self.max_tokens,
                system=system_with_cache,
                tools=tools_with_cache,
                messages=conversation_history,
                call_type="initial"
            )

        # Call Claude API with cached system + tools
        response = self.client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=system_with_cache,  # type: ignore
            tools=tools_with_cache,  # type: ignore
            messages=conversation_history
        )

        # Log API response (after receiving the response)
        if self.logger:
            self.logger.log_api_response(response, call_type="initial")

        # Process response in a loop to handle multi-turn tool calling
        all_tool_results = []
        panorama_base64 = None
        assistant_text_parts = []
        total_input_tokens = 0
        total_output_tokens = 0
        total_cache_creation_tokens = 0
        total_cache_read_tokens = 0

        # Continue until we get a response with no tool calls
        while True:
            # Track token usage from this response
            if hasattr(response, 'usage'):
                total_input_tokens += response.usage.input_tokens
                total_output_tokens += response.usage.output_tokens

                # Track cache usage if available
                if hasattr(response.usage, 'cache_creation_input_tokens'):
                    total_cache_creation_tokens += response.usage.cache_creation_input_tokens or 0
                if hasattr(response.usage, 'cache_read_input_tokens'):
                    total_cache_read_tokens += response.usage.cache_read_input_tokens or 0
            assistant_content = []
            current_tool_results = []

            # Extract text and tool uses from current response
            for block in response.content:
                if block.type == "text":
                    assistant_text_parts.append(block.text)
                    assistant_content.append(block)
                elif block.type == "tool_use":
                    # Execute the tool
                    tool_result = self.execute_tool(block.name, block.input)
                    current_tool_results.append({
                        "tool_name": block.name,
                        "tool_input": block.input,
                        "result": tool_result,
                        "tool_use_id": block.id
                    })
                    all_tool_results.append({
                        "tool_name": block.name,
                        "tool_input": block.input,
                        "result": tool_result
                    })

                    # Save panorama if this was a get_panorama call
                    if block.name == "get_panorama" and tool_result.get("success"):
                        panorama_base64 = tool_result.get("panorama_base64")

                    assistant_content.append(block)

            # Add assistant response to history
            conversation_history.append({
                "role": "assistant",
                "content": assistant_content
            })

            # If no tools were used, we're done
            if not current_tool_results:
                break

            # Build tool result messages
            tool_result_content = []
            for tool_result_info in current_tool_results:
                result_to_send = tool_result_info["result"].copy()

                # Remove large base64 panorama from result to avoid token explosion
                # Replace it with a simple confirmation message
                if tool_result_info["tool_name"] == "get_panorama" and "panorama_base64" in result_to_send:
                    result_to_send["panorama_base64"] = "[IMAGE DATA REMOVED - panorama captured and displayed to user]"

                tool_result_content.append({
                    "type": "tool_result",
                    "tool_use_id": tool_result_info["tool_use_id"],
                    "content": json.dumps(result_to_send)
                })

            conversation_history.append({
                "role": "user",
                "content": tool_result_content
            })

            # Log API request (before making the follow-up call)
            if self.logger:
                self.logger.log_api_request(
                    model=self.model,
                    max_tokens=self.max_tokens,
                    system=system_with_cache,
                    tools=tools_with_cache,
                    messages=conversation_history,
                    call_type="follow-up"
                )

            # Get follow-up response from Claude (with caching)
            response = self.client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                system=system_with_cache,  # type: ignore
                tools=tools_with_cache,  # type: ignore
                messages=conversation_history
            )

            # Log API response (after receiving the follow-up response)
            if self.logger:
                self.logger.log_api_response(response, call_type="follow-up")

        assistant_response = "\n".join(assistant_text_parts)

        # Log cache performance if caching was used
        if self.logger and (total_cache_creation_tokens > 0 or total_cache_read_tokens > 0):
            self.logger.console_info(
                f"Cache: {total_cache_read_tokens} tokens read, {total_cache_creation_tokens} tokens written"
            )

        if self.logger:
            self.logger.log_interactive_message("assistant", assistant_response, total_input_tokens, total_output_tokens)

        token_usage = {
            "input_tokens": total_input_tokens,
            "output_tokens": total_output_tokens,
            "cache_creation_input_tokens": total_cache_creation_tokens,
            "cache_read_input_tokens": total_cache_read_tokens
        }

        return assistant_response, all_tool_results, panorama_base64, token_usage
