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

        # Load prompt templates
        self.planning_system_prompt = self._load_file('planning_system_prompt.txt')
        self.planning_user_prompt = self._load_file('planning_user_prompt.txt')
        self.review_system_prompt = self._load_file('review_system_prompt.txt')
        self.review_user_prompt = self._load_file('review_user_prompt.txt')
        self.refinement_system_prompt = self._load_file('refinement_system_prompt.txt')
        self.refinement_user_prompt = self._load_file('refinement_user_prompt.txt')

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

    def _build_objects_info(self) -> tuple[str, str]:
        """
        Build objects list and info strings.

        Returns:
            Tuple of (objects_list, objects_info)
        """
        objects = self.object_manager.objects

        # Build objects list
        objects_list = "- " + "\n- ".join([f"{name}" for name in objects.keys()])

        # Build objects info with positions and dimensions
        objects_info_parts = []
        for name, obj_id in objects.items():
            pos = self.object_manager.get_object_center_position(name)
            dims = self.object_manager.get_object_dimensions(obj_id)
            objects_info_parts.append(
                f"- {name}:\n"
                f"  - Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] meters\n"
                f"  - Dimensions: [{dims[0]:.3f}, {dims[1]:.3f}, {dims[2]:.3f}] meters (width, depth, height)"
            )
        objects_info = "\n".join(objects_info_parts)

        return objects_list, objects_info

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

    def _generate_initial_plan(self, task_description: str, panorama_path: str) -> Dict[str, Any]:
        """
        Generate initial plan using LangChain.

        Args:
            task_description: The task to accomplish
            panorama_path: Path to panorama image

        Returns:
            Parsed plan dictionary
        """
        if self.logger:
            self.logger.console_info("Generating initial plan...")

        # Build object info
        objects_list, objects_info = self._build_objects_info()

        # Format system prompt
        system_prompt = self.planning_system_prompt.format(
            OBJECTS_LIST=objects_list,
            OBJECTS_INFO=objects_info
        )

        # Format user prompt
        user_prompt_text = self.planning_user_prompt.format(
            TASK_DESCRIPTION=task_description
        )

        # Encode image
        image_data = self._encode_image(panorama_path)

        # Create user message with image and text
        user_content = [
            {
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": self._get_image_media_type(panorama_path),
                    "data": image_data,
                },
            },
            {
                "type": "text",
                "text": user_prompt_text
            }
        ]

        # Call Anthropic API
        if self.logger:
            self.logger.log_llm_request(system_prompt, user_prompt_text, self.model)

        response = self.client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=system_prompt,
            messages=[
                {
                    "role": "user",
                    "content": user_content
                }
            ]
        )
        # Extract text from response content blocks
        response_text = ""
        for block in response.content:
            if block.type == "text":
                response_text += block.text

        # Extract token usage
        input_tokens = response.usage.input_tokens if hasattr(response, 'usage') else None
        output_tokens = response.usage.output_tokens if hasattr(response, 'usage') else None

        if self.logger:
            self.logger.log_llm_response(response_text, self.model, input_tokens, output_tokens)

        # Parse JSON
        plan = self._parse_json_response(response_text)

        return plan

    def _critique_plan(self, plan: Dict[str, Any], task_description: str, panorama_path: str) -> Dict[str, Any]:
        """
        Critique a plan using LangChain.

        Args:
            plan: The plan to critique
            task_description: The original task
            panorama_path: Path to panorama image

        Returns:
            Critique dictionary with is_valid, critique, suggestions
        """
        if self.logger:
            self.logger.console_info("Reviewing plan...")

        # Build object info
        objects_list, objects_info = self._build_objects_info()

        # Format user prompt
        user_prompt_text = self.review_user_prompt.format(
            TASK_DESCRIPTION=task_description,
            OBJECTS_INFO=objects_info,
            PLAN=json.dumps(plan, indent=2)
        )

        # Encode image
        image_data = self._encode_image(panorama_path)

        # Create user message with image and text
        user_content = [
            {
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": self._get_image_media_type(panorama_path),
                    "data": image_data,
                },
            },
            {
                "type": "text",
                "text": user_prompt_text
            }
        ]

        # Call Anthropic API
        if self.logger:
            self.logger.log_llm_request(self.review_system_prompt, user_prompt_text, self.model)

        response = self.client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=self.review_system_prompt,
            messages=[
                {
                    "role": "user",
                    "content": user_content
                }
            ]
        )
        # Extract text from response content blocks
        response_text = ""
        for block in response.content:
            if block.type == "text":
                response_text += block.text

        # Extract token usage
        input_tokens = response.usage.input_tokens if hasattr(response, 'usage') else None
        output_tokens = response.usage.output_tokens if hasattr(response, 'usage') else None

        if self.logger:
            self.logger.log_llm_response(response_text, self.model, input_tokens, output_tokens)

        # Parse JSON
        critique = self._parse_json_response(response_text)

        return critique

    def _refine_plan(self, plan: Dict[str, Any], critique: Dict[str, Any], task_description: str, panorama_path: str) -> Dict[str, Any]:
        """
        Refine a plan based on critique using LangChain.

        Args:
            plan: The original plan
            critique: The critique feedback
            task_description: The original task
            panorama_path: Path to panorama image

        Returns:
            Refined plan dictionary
        """
        if self.logger:
            self.logger.console_info("Refining plan based on review feedback...")

        # Build object info
        objects_list, objects_info = self._build_objects_info()

        # Format system prompt
        system_prompt = self.refinement_system_prompt.format(
            OBJECTS_LIST=objects_list,
            OBJECTS_INFO=objects_info
        )

        # Format critique text
        critique_text = f"Critique: {critique.get('critique', '')}\n\nSuggestions:\n"
        for i, suggestion in enumerate(critique.get('suggestions', []), 1):
            critique_text += f"{i}. {suggestion}\n"

        # Format user prompt
        user_prompt_text = self.refinement_user_prompt.format(
            TASK_DESCRIPTION=task_description,
            OBJECTS_INFO=objects_info,
            ORIGINAL_PLAN=json.dumps(plan, indent=2),
            CRITIQUE=critique_text
        )

        # Encode image
        image_data = self._encode_image(panorama_path)

        # Create user message with image and text
        user_content = [
            {
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": self._get_image_media_type(panorama_path),
                    "data": image_data,
                },
            },
            {
                "type": "text",
                "text": user_prompt_text
            }
        ]

        # Call Anthropic API
        if self.logger:
            self.logger.log_llm_request(system_prompt, user_prompt_text, self.model)

        response = self.client.messages.create(
            model=self.model,
            max_tokens=self.max_tokens,
            system=system_prompt,
            messages=[
                {
                    "role": "user",
                    "content": user_content
                }
            ]
        )
        # Extract text from response content blocks
        response_text = ""
        for block in response.content:
            if block.type == "text":
                response_text += block.text

        # Extract token usage
        input_tokens = response.usage.input_tokens if hasattr(response, 'usage') else None
        output_tokens = response.usage.output_tokens if hasattr(response, 'usage') else None

        if self.logger:
            self.logger.log_llm_response(response_text, self.model, input_tokens, output_tokens)

        # Parse JSON
        refined_plan = self._parse_json_response(response_text)

        return refined_plan

    def get_validated_plan(self, task_description: str, panorama_path: str, max_iterations: Optional[int] = None) -> Dict[str, Any]:
        """
        Generate and validate a plan through iterative critique and refinement.

        This is the main entry point that orchestrates the entire validation workflow:
        1. Generate initial plan
        2. Critique the plan
        3. If not valid, refine based on critique
        4. Repeat critique-refinement cycle until valid or max iterations reached
        5. Return final validated plan

        Args:
            task_description: The task to accomplish
            panorama_path: Path to panorama image
            max_iterations: Maximum critique-refinement cycles (default from config)

        Returns:
            Final validated plan dictionary
        """
        if max_iterations is None:
            max_iterations = config.MAX_VALIDATION_ITERATIONS if hasattr(config, 'MAX_VALIDATION_ITERATIONS') else 3

        # Step 1: Generate initial plan
        current_plan = self._generate_initial_plan(task_description, panorama_path)

        # Step 2-N: Critique and refine loop
        for iteration in range(max_iterations):

            # Critique the current plan
            critique = self._critique_plan(current_plan, task_description, panorama_path)

            # Check if plan is valid
            if critique.get('is_valid', False):
                if self.logger:
                    self.logger.log_app_plan_validated(f"Valid after {iteration + 1} iteration(s)")
                    self.logger.console_info(f">> Plan validated successfully (iteration {iteration + 1})")

                return current_plan

            # Plan needs refinement
            if self.logger:
                self.logger.log_app_warning(f"Plan validation iteration {iteration + 1}: Issues found")
                self.logger.console_info(f">> Plan needs refinement (iteration {iteration + 1}/{max_iterations})")

            # If this is the last iteration, return current plan with warning
            if iteration == max_iterations - 1:
                if self.logger:
                    self.logger.log_app_warning(f"Max validation iterations ({max_iterations}) reached. Using best attempt.")
                    self.logger.console_info(f"Plan validation completed (max iterations reached)")
                return current_plan

            # Refine the plan
            current_plan = self._refine_plan(current_plan, critique, task_description, panorama_path)

        # This should not be reached, but just in case
        return current_plan
