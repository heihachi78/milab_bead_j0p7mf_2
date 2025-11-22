import os
import json
import base64
from pathlib import Path
from typing import Dict, Any, Optional
from dotenv import load_dotenv

from langchain_anthropic import ChatAnthropic
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.messages import HumanMessage

from . import config


class LLMValidator:
    """LangChain-based validator for robot task plans with self-review and refinement."""

    def __init__(self, object_manager):
        """
        Initialize the LLM validator.

        Args:
            object_manager: ObjectManager instance for querying object positions
        """
        self.object_manager = object_manager

        # Load environment variables
        load_dotenv()
        self.api_key = os.getenv('ANTHROPIC_API_KEY')
        if not self.api_key:
            raise ValueError("ANTHROPIC_API_KEY not found in .env file")

        # Get model from config or environment
        self.model = config.VALIDATION_MODEL if hasattr(config, 'VALIDATION_MODEL') and config.VALIDATION_MODEL else os.getenv('ANTHROPIC_MODEL', 'claude-sonnet-4-5-20250929')

        # Initialize Anthropic chat model
        self.llm = ChatAnthropic(
            api_key=self.api_key,
            model=self.model,
            max_tokens=2048
        )

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
        Parse JSON from LLM response, handling markdown code blocks.

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
            json_str = response_text.strip()

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
        print(f"\n=== GENERATING INITIAL PLAN ===")
        print(f"Task: {task_description}")

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

        # Create message with image and text
        messages = [
            {
                "role": "system",
                "content": system_prompt
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/png",
                            "data": image_data,
                        },
                    },
                    {
                        "type": "text",
                        "text": user_prompt_text
                    }
                ]
            }
        ]

        # Call LLM
        print(f"Calling LLM for plan generation...")
        response = self.llm.invoke(messages)
        response_text = response.content

        print(f"\n--- INITIAL PLAN RESPONSE ---")
        print(response_text)
        print(f"--- END INITIAL PLAN RESPONSE ---\n")

        # Parse JSON
        plan = self._parse_json_response(response_text)

        print(f"Parsed initial plan:")
        print(json.dumps(plan, indent=2))

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
        print(f"\n=== CRITIQUING PLAN ===")

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

        # Create messages
        messages = [
            {
                "role": "system",
                "content": self.review_system_prompt
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/png",
                            "data": image_data,
                        },
                    },
                    {
                        "type": "text",
                        "text": user_prompt_text
                    }
                ]
            }
        ]

        # Call LLM
        print(f"Calling LLM for critique...")
        response = self.llm.invoke(messages)
        response_text = response.content

        print(f"\n--- CRITIQUE RESPONSE ---")
        print(response_text)
        print(f"--- END CRITIQUE RESPONSE ---\n")

        # Parse JSON
        critique = self._parse_json_response(response_text)

        print(f"Parsed critique:")
        print(json.dumps(critique, indent=2))

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
        print(f"\n=== REFINING PLAN ===")

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

        # Create messages
        messages = [
            {
                "role": "system",
                "content": system_prompt
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": "image/png",
                            "data": image_data,
                        },
                    },
                    {
                        "type": "text",
                        "text": user_prompt_text
                    }
                ]
            }
        ]

        # Call LLM
        print(f"Calling LLM for refinement...")
        response = self.llm.invoke(messages)
        response_text = response.content

        print(f"\n--- REFINEMENT RESPONSE ---")
        print(response_text)
        print(f"--- END REFINEMENT RESPONSE ---\n")

        # Parse JSON
        refined_plan = self._parse_json_response(response_text)

        print(f"Parsed refined plan:")
        print(json.dumps(refined_plan, indent=2))

        return refined_plan

    def get_validated_plan(self, task_description: str, panorama_path: str, max_iterations: Optional[int] = None) -> Dict[str, Any]:
        """
        Generate and validate a plan through iterative critique and refinement.

        This is the main entry point that orchestrates the entire LangChain workflow:
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

        print(f"\n{'='*60}")
        print(f"=== LLM VALIDATION WORKFLOW START ===")
        print(f"{'='*60}")
        print(f"Task: {task_description}")
        print(f"Max iterations: {max_iterations}")

        # Step 1: Generate initial plan
        current_plan = self._generate_initial_plan(task_description, panorama_path)

        # Step 2-N: Critique and refine loop
        for iteration in range(max_iterations):
            print(f"\n{'='*60}")
            print(f"=== VALIDATION ITERATION {iteration + 1}/{max_iterations} ===")
            print(f"{'='*60}")

            # Critique the current plan
            critique = self._critique_plan(current_plan, task_description, panorama_path)

            # Check if plan is valid
            if critique.get('is_valid', False):
                print(f"\n{'='*60}")
                print(f"=== PLAN VALIDATED SUCCESSFULLY ===")
                print(f"{'='*60}")
                print(f"Iterations needed: {iteration + 1}")
                print(f"\nFinal plan:")
                print(json.dumps(current_plan, indent=2))
                return current_plan

            # Plan needs refinement
            print(f"\nPlan not valid. Issues found:")
            print(f"Critique: {critique.get('critique', 'N/A')}")
            print(f"Suggestions: {critique.get('suggestions', [])}")

            # If this is the last iteration, return current plan with warning
            if iteration == max_iterations - 1:
                print(f"\n{'='*60}")
                print(f"=== MAX ITERATIONS REACHED ===")
                print(f"{'='*60}")
                print(f"WARNING: Plan could not be validated within {max_iterations} iterations.")
                print(f"Returning best attempt plan.")
                print(f"\nFinal plan:")
                print(json.dumps(current_plan, indent=2))
                return current_plan

            # Refine the plan
            current_plan = self._refine_plan(current_plan, critique, task_description, panorama_path)

        # This should not be reached, but just in case
        return current_plan
