"""
Multi-channel logging system for robot simulation.

Provides structured logging to separate files for LLM interactions,
robot actions, and application events. Includes Rich console output
for real-time progress display.
"""

import logging
import os
import json
from datetime import datetime
from typing import Optional, List, Dict, Any


class SimulationLogger:
    """
    Multi-channel logging system for robot simulation.

    Provides separate log files for:
    - LLM interactions (requests/responses)
    - Robot actions (movements, pick/place operations)
    - Application events (state changes, errors)

    Console output is limited to high-level progress messages.
    """

    def __init__(self, log_dir: str = "logs", session_name: Optional[str] = None):
        """
        Initialize the logging system with separate file handlers.

        Args:
            log_dir: Directory to store log files
            session_name: Optional session identifier (defaults to timestamp)
        """
        self.log_dir = log_dir

        # Create logs directory if it doesn't exist
        os.makedirs(log_dir, exist_ok=True)

        # Generate session identifier
        if session_name is None:
            session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_name = session_name

        # Initialize loggers
        self.llm_logger = self._create_logger(
            "llm",
            f"{log_dir}/llm_{session_name}.log",
            logging.DEBUG
        )

        self.robot_logger = self._create_logger(
            "robot",
            f"{log_dir}/robot_{session_name}.log",
            logging.DEBUG
        )

        self.app_logger = self._create_logger(
            "app",
            f"{log_dir}/app_{session_name}.log",
            logging.INFO
        )

        # Interactive mode logger (for chat conversations and tool calls)
        self.interactive_logger = self._create_logger(
            "interactive",
            f"{log_dir}/interactive_{session_name}.log",
            logging.DEBUG
        )

        # API call logger for detailed request/response logging (interactive mode only)
        # Check if API call logging is enabled in config
        from src import config
        if hasattr(config, 'LOG_API_CALLS') and config.LOG_API_CALLS:
            log_filename = getattr(config, 'API_CALLS_LOG_FILE', 'api_calls')
            self.api_call_logger = self._create_logger(
                "api_calls",
                f"{log_dir}/{log_filename}_{session_name}.log",
                logging.DEBUG
            )
        else:
            self.api_call_logger = None

        # RAG logger for query/result logging
        rag_log_filename = getattr(config, 'RAG_LOG_FILE', 'rag_queries.log')
        self.rag_logger = self._create_logger(
            "rag",
            f"{log_dir}/{rag_log_filename}",
            logging.DEBUG
        )

        # Console logger for important messages only
        self.console_logger = self._create_console_logger()

        self.log_app_info(f"Logging session started: {session_name}")

    def _create_logger(self, name: str, filepath: str, level: int) -> logging.Logger:
        """
        Create a logger with file handler.

        Args:
            name: Logger name identifier
            filepath: Path to log file
            level: Logging level (e.g., logging.DEBUG, logging.INFO)

        Returns:
            Configured logging.Logger instance
        """
        logger = logging.getLogger(f"sim.{name}")
        logger.setLevel(level)
        logger.propagate = False

        # Remove existing handlers
        logger.handlers.clear()

        # File handler with detailed formatting
        file_handler = logging.FileHandler(filepath, mode='w', encoding='utf-8')
        file_handler.setLevel(level)

        formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        return logger

    def _create_console_logger(self) -> logging.Logger:
        """
        Create console logger for important messages only.

        Returns:
            Configured logging.Logger instance for console output
        """
        logger = logging.getLogger("sim.console")
        logger.setLevel(logging.INFO)
        logger.propagate = False

        # Remove existing handlers
        logger.handlers.clear()

        # Console handler with simple formatting
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        formatter = logging.Formatter('%(message)s')
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

        return logger

    # ============== LLM Logging Methods ==============

    def log_llm_request(self, system_prompt: str, user_prompt: str, model: str = "unknown", stage: str = None):
        """
        Log an LLM request with system and user prompts.

        Args:
            system_prompt: System prompt text
            user_prompt: User prompt text
            model: Model name
            stage: Optional stage identifier (e.g., "SIMPLIFIED_PLANNING", "interactive")
        """
        self.llm_logger.info("=" * 80)
        stage_info = f" | Stage: {stage}" if stage else ""
        self.llm_logger.info(f"LLM REQUEST | Model: {model}{stage_info}")
        self.llm_logger.info("-" * 80)
        self.llm_logger.info("SYSTEM PROMPT:")
        self.llm_logger.info(system_prompt)
        self.llm_logger.info("-" * 80)
        self.llm_logger.info("USER PROMPT:")
        self.llm_logger.info(user_prompt)
        self.llm_logger.info("=" * 80)

    def log_llm_response(self, response: str, model: str = "unknown", input_tokens: int = None, output_tokens: int = None, stage: str = None, cache_creation_tokens: int = 0, cache_read_tokens: int = 0):
        """
        Log an LLM response with optional token usage and cache metrics.

        Args:
            response: Response text
            model: Model name
            input_tokens: Input token count
            output_tokens: Output token count
            stage: Optional stage identifier
            cache_creation_tokens: Cache creation token count
            cache_read_tokens: Cache read token count
        """
        self.llm_logger.info("=" * 80)
        stage_info = f" | Stage: {stage}" if stage else ""
        self.llm_logger.info(f"LLM RESPONSE | Model: {model}{stage_info}")
        if input_tokens is not None and output_tokens is not None:
            total_tokens = input_tokens + output_tokens
            self.llm_logger.info(f"TOKEN USAGE | Input: {input_tokens} | Output: {output_tokens} | Total: {total_tokens}")
            if cache_creation_tokens > 0 or cache_read_tokens > 0:
                self.llm_logger.info(f"CACHE USAGE | Created: {cache_creation_tokens} | Read: {cache_read_tokens}")
        self.llm_logger.info("-" * 80)
        self.llm_logger.info(response)
        self.llm_logger.info("=" * 80)
        self.llm_logger.info("")  # Empty line for readability

    # ============== Robot Logging Methods ==============

    def log_robot_operation_start(self, operation: str, **params):
        """
        Log the start of a robot operation with all parameters.

        Args:
            operation: Name of the operation (e.g., "pick_up", "place", "move_to_target")
            **params: All parameters relevant to this operation
        """
        self.robot_logger.info("=" * 60)
        self.robot_logger.info(f"OPERATION START: {operation}")
        if params:
            for key, value in params.items():
                if isinstance(value, (list, tuple)) and len(value) == 3:
                    self.robot_logger.info(f"  {key}: {self._format_pos(list(value))}")
                else:
                    self.robot_logger.info(f"  {key}: {value}")
        self.robot_logger.info("-" * 60)

    def log_robot_operation_end(self, operation: str, success: bool = True):
        """
        Log the end of a robot operation.

        Args:
            operation: Name of the operation
            success: Whether the operation completed successfully
        """
        status = "SUCCESS" if success else "FAILED"
        self.robot_logger.info(f"OPERATION END: {operation} | Status: {status}")
        self.robot_logger.info("=" * 60)
        self.robot_logger.info("")  # Empty line for readability

    def log_robot_pick(self, object_name: str, position: list):
        """Log pick operation."""
        self.robot_logger.info(
            f"PICK | Object: {object_name} | Position: {self._format_pos(position)}"
        )
        self.console_logger.info(f"Picking up {object_name}")

    def log_robot_place(self, position: list, on_object: Optional[str] = None):
        """Log place operation."""
        if on_object:
            self.robot_logger.info(
                f"PLACE | On: {on_object} | Position: {self._format_pos(position)}"
            )
            self.console_logger.info(f"Placing on {on_object}")
        else:
            self.robot_logger.info(
                f"PLACE | Position: {self._format_pos(position)}"
            )
            self.console_logger.info(f"Placing at position {self._format_pos(position)}")

    def log_robot_gripper(self, state: str, position: float):
        """Log gripper state change."""
        self.robot_logger.info(f"GRIPPER | State: {state} | Position: {position:.4f}")

    def log_robot_stabilize(self, steps: int):
        """Log stabilization operation."""
        self.robot_logger.info(f"STABILIZE | Steps: {steps}")
        self.console_logger.info("Stabilizing robot...")

    def log_robot_convergence(self, iterations: int, final_distance: float):
        """Log movement convergence."""
        self.robot_logger.debug(
            f"CONVERGED | Iterations: {iterations} | Final distance: {final_distance:.6f}m"
        )

    # ============== Application Logging Methods ==============

    def log_app_info(self, message: str):
        """Log general application information."""
        self.app_logger.info(message)

    def log_app_warning(self, message: str):
        """Log application warning."""
        self.app_logger.warning(message)
        self.console_logger.warning(f"WARNING: {message}")

    def log_app_error(self, message: str):
        """Log application error."""
        self.app_logger.error(message)
        self.console_logger.error(f"ERROR: {message}")

    def log_app_simulation_start(self, time: float):
        """Log simulation start."""
        self.app_logger.info("=" * 80)
        self.app_logger.info(f"SIMULATION START | t={time}")
        self.app_logger.info("=" * 80)
        self.console_logger.info("=" * 40)
        self.console_logger.info(f"Simulation started (t={time})")
        self.console_logger.info("=" * 40)

    def log_app_simulation_end(self, time: float):
        """Log simulation end."""
        self.app_logger.info("=" * 80)
        self.app_logger.info(f"SIMULATION END | t={time}")
        self.app_logger.info("=" * 80)
        self.console_logger.info("=" * 40)
        self.console_logger.info(f"Simulation ended (t={time})")
        self.console_logger.info("=" * 40)

    def log_app_plan_validated(self, validation_result: str):
        """Log plan validation result."""
        self.app_logger.info(f"PLAN VALIDATED: {validation_result}")

    def log_app_object_loaded(self, object_name: str, position: list):
        """Log object loading."""
        self.app_logger.info(
            f"OBJECT LOADED | Name: {object_name} | Position: {self._format_pos(position)}"
        )

    def log_app_camera_capture(self, capture_type: str, filename: str):
        """Log camera capture."""
        self.app_logger.info(f"CAMERA CAPTURE | Type: {capture_type} | File: {filename}")

    # ============== Console-only Methods ==============

    def console_info(self, message: str):
        """Print important information to console only."""
        self.console_logger.info(message)

    def console_progress(self, message: str):
        """Print progress update to console."""
        self.console_logger.info(f">> {message}")

    def console_success(self, message: str):
        """Print success message to console."""
        self.console_logger.info(message)

    def console_warning(self, message: str):
        """Print warning message to console."""
        self.console_logger.warning(message)

    def console_error(self, message: str):
        """Print error message to console."""
        self.console_logger.error(message)

    # ============== Interactive Mode Logging Methods ==============

    def log_interactive_message(self, role: str, content: str, input_tokens: int = None, output_tokens: int = None):
        """
        Log an interactive chat message with optional token usage.

        Args:
            role: Message role ("user" or "assistant")
            content: Message content
            input_tokens: Optional input token count (for assistant messages)
            output_tokens: Optional output token count (for assistant messages)
        """
        self.interactive_logger.info("=" * 60)
        self.interactive_logger.info(f"{role.upper()} MESSAGE")
        if role == "assistant" and input_tokens is not None and output_tokens is not None:
            total_tokens = input_tokens + output_tokens
            self.interactive_logger.info(f"TOKEN USAGE | Input: {input_tokens} | Output: {output_tokens} | Total: {total_tokens}")
        self.interactive_logger.info("-" * 60)
        self.interactive_logger.info(content)
        self.interactive_logger.info("=" * 60)
        self.interactive_logger.info("")  # Empty line for readability

    def log_tool_call(self, tool_name: str, parameters: dict):
        """
        Log a tool call from the LLM.

        Args:
            tool_name: Name of the tool being called
            parameters: Tool input parameters
        """
        self.interactive_logger.info("-" * 60)
        self.interactive_logger.info(f"TOOL CALL: {tool_name}")
        if parameters:
            import json
            self.interactive_logger.info(f"Parameters: {json.dumps(parameters, indent=2)}")
        self.interactive_logger.info("-" * 60)

    def log_tool_result(self, tool_name: str, result: dict):
        """
        Log the result of a tool call.

        Args:
            tool_name: Name of the tool that was executed
            result: Tool execution result
        """
        import json
        success = result.get("success", False)
        status = "SUCCESS" if success else "FAILED"

        self.interactive_logger.info(f"TOOL RESULT: {tool_name} | Status: {status}")

        # Log full result (but truncate panorama data if present)
        result_copy = result.copy()
        if "panorama_base64" in result_copy:
            result_copy["panorama_base64"] = f"<base64 data: {len(result_copy['panorama_base64'])} chars>"

        self.interactive_logger.info(f"Result: {json.dumps(result_copy, indent=2)}")
        self.interactive_logger.info("-" * 60)
        self.interactive_logger.info("")  # Empty line for readability

    # ============== API Call Logging Methods (Interactive Mode) ==============

    def log_api_request(self, model: str, max_tokens: int, system: List[Dict[str, Any]],
                       tools: List[Dict[str, Any]], messages: List[Dict[str, Any]],
                       call_type: str = "initial"):
        """
        Log complete API request details for debugging and analysis.

        Args:
            model: Model name being called
            max_tokens: Max tokens parameter
            system: System prompt with cache control
            tools: Tool definitions with cache control
            messages: Conversation history
            call_type: "initial" or "follow-up" to distinguish call types
        """
        if not self.api_call_logger:
            return

        self.api_call_logger.info("=" * 80)
        self.api_call_logger.info(f"API REQUEST | Type: {call_type} | Model: {model}")
        self.api_call_logger.info("-" * 80)
        self.api_call_logger.info("REQUEST PARAMETERS:")

        # Convert messages to serializable format
        serializable_messages = []
        for msg in messages:
            serializable_msg = {"role": msg["role"]}

            # Handle content field which may contain objects
            if "content" in msg:
                content = msg["content"]
                if isinstance(content, str):
                    # Simple string content
                    serializable_msg["content"] = content
                elif isinstance(content, list):
                    # List of content blocks (may be dicts or objects)
                    serializable_content = []
                    for item in content:
                        if isinstance(item, dict):
                            serializable_content.append(item)
                        else:
                            # It's an object (like TextBlock or ToolUseBlock)
                            try:
                                if hasattr(item, 'model_dump'):
                                    serializable_content.append(item.model_dump())
                                elif hasattr(item, 'dict'):
                                    serializable_content.append(item.dict())
                                else:
                                    # Manual conversion
                                    item_dict = {"type": getattr(item, 'type', 'unknown')}
                                    if hasattr(item, 'text'):
                                        item_dict["text"] = item.text
                                    if hasattr(item, 'id'):
                                        item_dict["id"] = item.id
                                    if hasattr(item, 'name'):
                                        item_dict["name"] = item.name
                                    if hasattr(item, 'input'):
                                        item_dict["input"] = item.input
                                    serializable_content.append(item_dict)
                            except Exception as e:
                                serializable_content.append({
                                    "type": "error_serializing",
                                    "error": str(e),
                                    "object_type": type(item).__name__
                                })
                    serializable_msg["content"] = serializable_content
                else:
                    serializable_msg["content"] = str(content)

            serializable_messages.append(serializable_msg)

        # Build request data structure
        request_data = {
            "model": model,
            "max_tokens": max_tokens,
            "system": system,
            "tools": tools,
            "messages": serializable_messages
        }

        # Pretty-print JSON with 2-space indentation
        formatted_request = json.dumps(request_data, indent=2, ensure_ascii=False)
        self.api_call_logger.info(formatted_request)
        self.api_call_logger.info("=" * 80)
        self.api_call_logger.info("")  # Empty line for readability

    def log_api_response(self, response: Any, call_type: str = "initial"):
        """
        Log complete API response details for debugging and analysis.

        Args:
            response: Full response object from Anthropic API
            call_type: "initial" or "follow-up" to distinguish call types
        """
        if not self.api_call_logger:
            return

        self.api_call_logger.info("=" * 80)
        self.api_call_logger.info(f"API RESPONSE | Type: {call_type}")
        self.api_call_logger.info("-" * 80)
        self.api_call_logger.info("RESPONSE CONTENT:")

        # Extract response data
        response_data = {
            "id": getattr(response, 'id', None),
            "model": getattr(response, 'model', None),
            "stop_reason": getattr(response, 'stop_reason', None),
            "content": [],
            "usage": {}
        }

        # Extract content blocks
        if hasattr(response, 'content'):
            for block in response.content:
                try:
                    if hasattr(block, 'model_dump'):
                        # Pydantic models have model_dump()
                        response_data["content"].append(block.model_dump())
                    elif hasattr(block, 'dict'):
                        # Or dict() method
                        response_data["content"].append(block.dict())
                    else:
                        # Fallback: convert to dict manually
                        block_dict = {
                            "type": getattr(block, 'type', 'unknown'),
                        }
                        # Add type-specific fields
                        if hasattr(block, 'text'):
                            block_dict["text"] = block.text
                        if hasattr(block, 'id'):
                            block_dict["id"] = block.id
                        if hasattr(block, 'name'):
                            block_dict["name"] = block.name
                        if hasattr(block, 'input'):
                            block_dict["input"] = block.input
                        response_data["content"].append(block_dict)
                except Exception as e:
                    # If all else fails, log a simple representation
                    response_data["content"].append({
                        "type": "error_serializing_block",
                        "error": str(e),
                        "block_type": type(block).__name__
                    })

        # Extract usage information
        if hasattr(response, 'usage'):
            response_data["usage"] = {
                "input_tokens": getattr(response.usage, 'input_tokens', 0),
                "output_tokens": getattr(response.usage, 'output_tokens', 0),
                "cache_creation_input_tokens": getattr(response.usage, 'cache_creation_input_tokens', 0),
                "cache_read_input_tokens": getattr(response.usage, 'cache_read_input_tokens', 0)
            }

        # Pretty-print JSON with 2-space indentation
        formatted_response = json.dumps(response_data, indent=2, ensure_ascii=False)
        self.api_call_logger.info(formatted_response)
        self.api_call_logger.info("=" * 80)
        self.api_call_logger.info("")  # Empty line for readability

    # ============== Main Application Logging Methods ==============

    def log_scene_loaded(self, scene_name: str, description: str, object_names: list):
        """Log scene loading with metadata."""
        self.console_info(f"Loaded scene: {scene_name}")
        self.app_logger.info(f"Scene: {scene_name} - {description}")
        self.app_logger.info(f"Objects in scene: {', '.join(object_names)}")

    def log_scene_load_error(self, scene_name: str, error: str):
        """Log scene loading failure."""
        print(f"ERROR: Failed to load scene '{scene_name}': {error}")
        self.app_logger.error(f"Failed to load scene '{scene_name}': {error}")

    def log_images_folder_cleared(self, folder: str, file_count: int):
        """Log clearing of images folder."""
        self.app_logger.info(f"Cleared {file_count} files from {folder} folder")

    def log_realtime_simulation_set(self, enabled: bool):
        """Log real-time simulation setting."""
        self.app_logger.info(f"Real-time simulation: {enabled}")

    def log_object_loaded(self, obj_type: str, obj_name: str, position: list):
        """Log object loading into scene."""
        self.app_logger.info(f"Loaded {obj_type}: {obj_name} at {position}")

    def log_unknown_object_type(self, obj_type: str, obj_name: str):
        """Log unknown object type warning."""
        self.app_logger.warning(f"Unknown object type '{obj_type}' for object '{obj_name}', skipping")

    def log_objects_loaded_count(self, count: int):
        """Log total objects loaded."""
        self.console_info(f"Loaded {count} objects successfully")

    def log_stabilization_complete(self, steps: int):
        """Log stabilization loop completion."""
        self.app_logger.info(f"Stabilization loop completed: {steps} steps")

    def log_panorama_captured(self, name: str):
        """Log panorama capture."""
        self.console_info(f"{name} panorama captured")

    def log_api_key_missing(self):
        """Log missing API key error."""
        self.console_error("ANTHROPIC_API_KEY not found in environment variables")

    def log_llm_systems_initialized(self):
        """Log LLM controller and validator initialization."""
        self.app_logger.info("LLM controller and validator initialized")

    def log_task_description(self, description: str):
        """Log task description."""
        self.app_logger.info(f"Task description: {description[:100]}...")

    def log_validation_failed(self, final_critique: dict, commands: list):
        """Log validation failure with details."""
        self.console_error("=" * 80)
        self.console_error("VALIDATION FAILED - NO VALID PLAN FOUND")
        self.console_error("=" * 80)
        self.console_error("")
        self.console_error("The LLM could not generate a valid plan.")
        self.console_error("")

        if final_critique:
            self.console_error("Validation errors:")
            errors = final_critique.get('errors', [])
            if errors:
                for error in errors:
                    self.console_error(f"  - {error}")
            elif 'error' in final_critique:
                self.console_error(f"  - {final_critique['error']}")
            self.console_error("")

        if not commands or len(commands) == 0:
            self.console_error("The final plan contains NO COMMANDS (empty command list).")
            self.console_error("This usually means the LLM could not generate a valid command sequence.")
        else:
            self.console_error(f"The final plan has {len(commands)} command(s) but failed validation checks.")

        self.console_error("")
        self.console_error("Possible reasons:")
        self.console_error("  - The task may be impossible given the current scene configuration")
        self.console_error("  - Object positions may make the task infeasible")
        self.console_error("  - The LLM may be confused about the scene state")
        self.console_error("")
        self.console_error("ABORTING EXECUTION - No robot operations will be performed.")
        self.console_error("=" * 80)

    def log_empty_plan_warning(self):
        """Log warning for empty command list."""
        self.console_warning("=" * 80)
        self.console_warning("WARNING: Plan contains no commands")
        self.console_warning("=" * 80)
        self.console_warning("")
        self.console_warning("The validated plan has an empty command list.")
        self.console_warning("This may indicate that:")
        self.console_warning("  - The scene is already in the desired state")
        self.console_warning("  - No actions are needed to complete the task")
        self.console_warning("")
        self.console_warning("Skipping execution (nothing to do).")
        self.console_warning("=" * 80)

    def log_execution_starting(self, command_count: int):
        """Log execution start with command count."""
        self.console_info("Executing validated plan...")
        self.console_info(f"Plan contains {command_count} command(s)")

    def log_execution_exception(self, exception_message: str):
        """Log execution exception."""
        self.console_error("=" * 80)
        self.console_error("EXECUTION FAILED WITH EXCEPTION")
        self.console_error("=" * 80)
        self.console_error(f"Exception: {exception_message}")
        self.console_error("=" * 80)

    def log_execution_failed(self, reason: str, details: str = None):
        """Log execution failure."""
        self.console_error("=" * 80)
        self.console_error("EXECUTION FAILED")
        self.console_error("=" * 80)
        self.console_error(f"Reason: {reason}")
        if details:
            self.console_error(f"Details: {details}")
        self.console_error("=" * 80)

    def log_execution_success(self, steps_completed: int):
        """Log execution success."""
        self.console_success("=" * 80)
        self.console_success("EXECUTION COMPLETED SUCCESSFULLY")
        self.console_success(f"Completed {steps_completed} steps")
        self.console_success("=" * 80)

    def log_verification_header(self):
        """Log verification section header."""
        self.console_info("")
        self.console_info("=" * 80)
        self.console_info("POST-EXECUTION VERIFICATION")
        self.console_info("=" * 80)

    def log_verification_panorama_saved(self, path: str):
        """Log verification panorama save."""
        self.console_info(f"Post-execution panorama saved: {path}")

    def log_verification_results(self, verification_result: dict):
        """Log verification results."""
        self.console_info("")
        self.console_info("Verification Results:")
        self.console_info("-" * 80)

        task_satisfied = verification_result.get("task_satisfied")

        if task_satisfied is None:
            self.console_error(f"Status: VERIFICATION ERROR")
            self.console_error(f"Message: {verification_result.get('reasoning', 'Unknown error')}")
        elif task_satisfied:
            self.console_success(f"Status: TASK SATISFIED ✓")
            self.console_success(f"Reasoning: {verification_result.get('reasoning', 'No reasoning provided')}")
            self.console_info(f"Actual State: {verification_result.get('actual_state', 'Not provided')}")
            self.console_info(f"Expected State: {verification_result.get('expected_state', 'Not provided')}")
        else:
            self.console_warning(f"Status: TASK NOT SATISFIED ✗")
            self.console_warning(f"Reasoning: {verification_result.get('reasoning', 'No reasoning provided')}")
            self.console_info(f"Actual State: {verification_result.get('actual_state', 'Not provided')}")
            self.console_info(f"Expected State: {verification_result.get('expected_state', 'Not provided')}")

            discrepancies = verification_result.get("discrepancies")
            if discrepancies:
                self.console_warning("Discrepancies:")
                for discrepancy in discrepancies:
                    self.console_warning(f"  - {discrepancy}")

        self.console_info("=" * 80)
        self.console_info("")

    # ============== RAG Logging Methods ==============

    def log_rag_query(self, query: str, current_objects: Optional[List[str]] = None):
        """
        Log a RAG query.

        Args:
            query: The query text
            current_objects: Optional list of current object names
        """
        self.rag_logger.info("=" * 80)
        self.rag_logger.info("RAG QUERY")
        self.rag_logger.info("-" * 80)
        self.rag_logger.info(f"Query: {query}")
        if current_objects:
            self.rag_logger.info(f"Current objects: {', '.join(current_objects)}")
        self.rag_logger.info("-" * 80)

    def log_rag_results(self, examples: List[Dict[str, Any]], formatted_context: str):
        """
        Log RAG results.

        Args:
            examples: List of retrieved examples
            formatted_context: The formatted context string that will be injected
        """
        self.rag_logger.info(f"Results: {len(examples)} example(s) found")
        self.rag_logger.info("-" * 80)

        if not examples:
            self.rag_logger.info("No relevant examples found.")
        else:
            for i, example in enumerate(examples, 1):
                self.rag_logger.info(f"Example {i}:")
                self.rag_logger.info(f"  Task: {example.get('task_description', 'N/A')}")
                self.rag_logger.info(f"  Objects: {example.get('object_count', 0)} ({example.get('object_types', 'N/A')})")
                self.rag_logger.info(f"  Colors: {example.get('object_colors', 'N/A')}")
                if example.get('plan_reasoning'):
                    self.rag_logger.info(f"  Reasoning: {example['plan_reasoning'][:200]}...")
                commands = example.get('plan_commands', [])
                self.rag_logger.info(f"  Commands: {len(commands)} command(s)")

        self.rag_logger.info("-" * 80)
        self.rag_logger.info("FORMATTED CONTEXT:")
        self.rag_logger.info(formatted_context if formatted_context else "(empty)")
        self.rag_logger.info("=" * 80)
        self.rag_logger.info("")  # Empty line for readability

    # ============== Helper Methods ==============

    def _format_pos(self, position: list) -> str:
        """
        Format position array for logging.

        Args:
            position: Position list [x, y, z]

        Returns:
            Formatted position string with 3 decimal places
        """
        return f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]"

    def close(self):
        """Close all log handlers."""
        loggers_to_close = [self.llm_logger, self.robot_logger, self.app_logger, self.interactive_logger, self.console_logger, self.rag_logger]
        if self.api_call_logger:
            loggers_to_close.append(self.api_call_logger)

        for logger in loggers_to_close:
            for handler in logger.handlers:
                handler.close()
                logger.removeHandler(handler)
