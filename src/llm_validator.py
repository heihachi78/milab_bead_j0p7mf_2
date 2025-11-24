import os
import json
import base64
from pathlib import Path
from typing import Dict, Any, Optional
from dotenv import load_dotenv

import anthropic

from . import config


class LLMValidator:
    """Anthropic-based validator for robot task plans with self-review and refinement."""

    def __init__(self, object_manager, logger=None):
        """
        Initialize the LLM validator.

        Args:
            object_manager: ObjectManager instance for querying object positions
            logger: SimulationLogger instance for logging (optional)
        """
        self.object_manager = object_manager
        self.logger = logger

        # Load environment variables
        load_dotenv()
        self.api_key = os.getenv('ANTHROPIC_API_KEY')
        if not self.api_key:
            raise ValueError("ANTHROPIC_API_KEY not found in .env file")

        # Get model from config or environment
        self.model = config.VALIDATION_MODEL if hasattr(config, 'VALIDATION_MODEL') and config.VALIDATION_MODEL else os.getenv('ANTHROPIC_MODEL', 'claude-sonnet-4-5-20250929')

        # Initialize Anthropic client
        self.client = anthropic.Anthropic(api_key=self.api_key)
        self.max_tokens = 2048

        # Load simplified prompt templates
        self.simplified_system_prompt = self._load_file('simplified_system_prompt.txt')
        self.simplified_user_prompt = self._load_file('simplified_user_prompt.txt')

    def _load_file(self, filename: str) -> str:
        """Load text file content from prompts folder."""
        filepath = Path(__file__).parent.parent / 'prompts' / filename
        with open(filepath, 'r') as f:
            return f.read().strip()

    def _encode_image(self, image_path: str) -> str:
        """Encode image to base64."""
        with open(image_path, 'rb') as f:
            return base64.standard_b64encode(f.read()).decode('utf-8')

    def _get_image_media_type(self, image_path: str) -> str:
        """Determine media type based on file extension."""
        extension = Path(image_path).suffix.lower()
        if extension in ['.jpg', '.jpeg']:
            return 'image/jpeg'
        elif extension == '.png':
            return 'image/png'
        else:
            # Default to JPEG based on config
            return 'image/jpeg' if config.PANORAMA_FORMAT == 'JPEG' else 'image/png'

    def _build_objects_info(self) -> str:
        """
        Build detailed objects info string with positions, dimensions, and colors.

        Returns:
            Formatted objects info string
        """
        objects = self.object_manager.objects

        # Build objects info with positions, dimensions, and colors
        objects_info_parts = []
        for name, obj_id in objects.items():
            pos = self.object_manager.get_object_center_position(name)
            dims = self.object_manager.get_object_dimensions(obj_id)

            # Get color if available
            color_info = ""
            if hasattr(self.object_manager, 'object_colors') and name in self.object_manager.object_colors:
                rgba = self.object_manager.object_colors[name]
                color_name = self._get_color_name(rgba)
                color_info = f"  - Color: RGBA({rgba[0]:.1f}, {rgba[1]:.1f}, {rgba[2]:.1f}, {rgba[3]:.1f}) [{color_name}]\n"

            objects_info_parts.append(
                f"- {name}:\n"
                f"  - Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] meters\n"
                f"  - Dimensions: [{dims[0]:.3f}, {dims[1]:.3f}, {dims[2]:.3f}] meters (width, depth, height)\n"
                f"{color_info}"
            )
        objects_info = "\n".join(objects_info_parts)

        return objects_info

    def _get_color_name(self, rgba: list) -> str:
        """Convert RGBA values to human-readable color name."""
        r, g, b, _ = rgba  # Unpack but don't use alpha

        # Simple color name mapping
        if r > 0.9 and g < 0.1 and b < 0.1:
            return "Red"
        elif r < 0.1 and g > 0.9 and b < 0.1:
            return "Green"
        elif r < 0.1 and g < 0.1 and b > 0.9:
            return "Blue"
        elif r > 0.9 and g > 0.9 and b < 0.1:
            return "Yellow"
        elif r > 0.4 and r < 0.6 and g < 0.1 and b > 0.4 and b < 0.6:
            return "Purple"
        elif r > 0.9 and g > 0.4 and g < 0.6 and b > 0.9:
            return "Pink"
        elif r > 0.9 and g > 0.6 and b < 0.1:
            return "Orange"
        else:
            return "Custom"

    def _parse_json_response(self, response_text: str) -> Dict[str, Any]:
        """
        Parse JSON from LLM response, handling markdown code blocks and extra text.

        Args:
            response_text: Raw text response from LLM

        Returns:
            Parsed JSON dictionary
        """
        # Try to extract JSON from markdown code blocks if present
        if "```json" in response_text:
            json_start = response_text.find("```json") + 7
            json_end = response_text.find("```", json_start)
            json_str = response_text[json_start:json_end].strip()
        elif "```" in response_text:
            json_start = response_text.find("```") + 3
            json_end = response_text.find("```", json_start)
            json_str = response_text[json_start:json_end].strip()
        else:
            # Try to extract JSON object by finding the first { and matching }
            json_str = response_text.strip()
            if json_str.startswith('{'):
                # Count braces to find the end of the JSON object
                brace_count = 0
                for i, char in enumerate(json_str):
                    if char == '{':
                        brace_count += 1
                    elif char == '}':
                        brace_count -= 1
                        if brace_count == 0:
                            # Found the end of the JSON object
                            json_str = json_str[:i+1]
                            break

        return json.loads(json_str)

    def _validate_plan_locally(self, plan: Dict[str, Any], objects_info: str) -> tuple[bool, list[str]]:
        """
        Perform basic Python validation checks on the plan.

        Args:
            plan: The plan dictionary to validate
            objects_info: Object information for reference

        Returns:
            Tuple of (is_valid, errors) where errors is a list of validation error messages
        """
        errors = []

        # Check JSON structure
        if "reasoning" not in plan:
            errors.append("Plan missing 'reasoning' field")
        if "commands" not in plan:
            errors.append("Plan missing 'commands' field")
            return False, errors

        commands = plan.get("commands", [])
        if not isinstance(commands, list):
            errors.append("'commands' field must be a list")
            return False, errors

        # Track gripper state for logical sequence check
        holding_object = False

        # Validate each command
        for i, cmd in enumerate(commands):
            if not isinstance(cmd, dict):
                errors.append(f"Command {i+1} is not a dictionary")
                continue

            if "action" not in cmd:
                errors.append(f"Command {i+1} missing 'action' field")
                continue

            action = cmd.get("action")

            # Valid actions
            valid_actions = ["pick_up", "place", "place_on", "rotate_gripper_90", "reset_gripper_orientation"]
            if action not in valid_actions:
                errors.append(f"Command {i+1}: Invalid action '{action}'. Must be one of {valid_actions}")

            # Validate ground placement has z=0
            if action == "place":
                position = cmd.get("position")
                if not position:
                    errors.append(f"Command {i+1}: 'place' action requires 'position' field")
                elif isinstance(position, list) and len(position) >= 3:
                    if position[2] != 0:
                        errors.append(f"Command {i+1}: Ground placement must have z=0, got z={position[2]}")
                else:
                    errors.append(f"Command {i+1}: 'position' must be a list [x, y, z]")

            # Logical sequence check: can't place without picking first
            if action == "pick_up":
                if holding_object:
                    errors.append(f"Command {i+1}: Cannot pick_up while already holding an object")
                holding_object = True
            elif action in ["place", "place_on"]:
                if not holding_object:
                    errors.append(f"Command {i+1}: Cannot {action} without picking up an object first")
                holding_object = False

        is_valid = len(errors) == 0
        return is_valid, errors

    def _generate_plan(self, task_description: str) -> Dict[str, Any]:
        """
        Generate plan using simplified single-call approach.

        Args:
            task_description: The task to accomplish

        Returns:
            Plan dictionary with reasoning and commands
        """
        if self.logger:
            self.logger.console_info("Generating plan from scene data...")

        # Build object info with positions, dimensions, and colors
        objects_info = self._build_objects_info()

        # Prepare system prompt with caching
        system_with_cache = [
            {
                "type": "text",
                "text": self.simplified_system_prompt,
                "cache_control": {"type": "ephemeral"}  # Cache system prompt with examples
            }
        ]

        # Format user prompt with scene data
        user_prompt_text = self.simplified_user_prompt.format(
            objects_info=objects_info,
            task_description=task_description
        )

        # Call Anthropic API
        if self.logger:
            self.logger.log_llm_request(self.simplified_system_prompt, user_prompt_text, self.model, stage="SIMPLIFIED_PLANNING")

        response = self.client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=system_with_cache,  # type: ignore
            messages=[
                {
                    "role": "user",
                    "content": user_prompt_text
                }
            ]
        )

        # Extract text from response content blocks
        response_text = ""
        for block in response.content:
            if block.type == "text":
                response_text += block.text

        # Extract token usage (including cache metrics)
        input_tokens = response.usage.input_tokens if hasattr(response, 'usage') else None
        output_tokens = response.usage.output_tokens if hasattr(response, 'usage') else None
        cache_creation_tokens = (response.usage.cache_creation_input_tokens or 0) if hasattr(response, 'usage') and hasattr(response.usage, 'cache_creation_input_tokens') else 0
        cache_read_tokens = (response.usage.cache_read_input_tokens or 0) if hasattr(response, 'usage') and hasattr(response.usage, 'cache_read_input_tokens') else 0

        if self.logger:
            self.logger.log_llm_response(response_text, self.model, input_tokens, output_tokens, stage="SIMPLIFIED_PLANNING", cache_creation_tokens=cache_creation_tokens, cache_read_tokens=cache_read_tokens)
            self.logger.console_success("Plan generation complete")

        # Parse JSON
        plan = self._parse_json_response(response_text)
        return plan

    def get_validated_plan(self, task_description: str, panorama_path: Optional[str] = None, max_iterations: Optional[int] = None) -> tuple[Dict[str, Any], bool, Optional[Dict[str, Any]]]:
        """
        Generate and validate a plan using simplified single-call workflow.

        This method generates a plan from scene data using a single LLM call with
        comprehensive system prompt containing examples, then validates it locally.

        Args:
            task_description: The task to accomplish
            panorama_path: Path to panorama image (optional, kept for compatibility, still captured for debugging)
            max_iterations: Maximum iterations (unused in simplified mode, kept for compatibility)

        Returns:
            Tuple of (plan, is_valid, validation_errors_dict)
            - plan: Plan dictionary with reasoning and commands
            - is_valid: Boolean indicating if plan passed local validation
            - validation_errors_dict: Dictionary with errors if validation failed, None if valid
        """
        if self.logger:
            self.logger.console_info("Simplified workflow: Generating plan from scene data...")

        # Generate plan using single LLM call
        try:
            plan = self._generate_plan(task_description)
        except Exception as e:
            if self.logger:
                self.logger.log_app_error(f"Plan generation failed: {str(e)}")
                self.logger.console_error(f"Plan generation failed: {str(e)}")
            # Return empty plan with error
            return {"reasoning": "", "commands": []}, False, {"error": str(e)}

        # Perform local validation
        objects_info = self._build_objects_info()
        is_valid, errors = self._validate_plan_locally(plan, objects_info)

        if is_valid:
            if self.logger:
                self.logger.log_app_plan_validated("Plan validated successfully")
                self.logger.console_success("Plan validated successfully")
            return plan, True, None
        else:
            if self.logger:
                self.logger.log_app_error(f"Plan validation failed: {'; '.join(errors)}")
                self.logger.console_error("Plan validation failed:")
                for error in errors:
                    self.logger.console_error(f"  - {error}")
            return plan, False, {"errors": errors}
