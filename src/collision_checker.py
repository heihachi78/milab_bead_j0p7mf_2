"""
Collision prediction module for robot manipulation.
Checks if a planned step will cause collision before execution.
"""

import json
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from anthropic import Anthropic
from src.config import COLLISION_CHECK_MODEL, ENABLE_PROMPT_CACHING, COLLISION_CHECK_CACHE_TTL
from src.logger import SimulationLogger


@dataclass
class CollisionResult:
    """Result of collision prediction check."""
    will_collide: bool
    collision_type: str  # "none", "gripper-object", "arm-object", "object-object"
    risk_level: str  # "none", "low", "medium", "high"
    affected_objects: List[str]
    reasoning: str


class CollisionChecker:
    """
    Checks if a robot action will cause collision using LLM reasoning.
    Uses text-only prompts with object positions and dimensions.
    """

    def __init__(self, anthropic_client: Anthropic, object_manager, robot_controller, logger: SimulationLogger):
        """
        Initialize collision checker.

        Args:
            anthropic_client: Anthropic API client
            object_manager: ObjectManager instance for object data
            robot_controller: RobotController instance for gripper state
            logger: SimulationLogger instance for logging
        """
        self.client = anthropic_client
        self.object_manager = object_manager
        self.robot_controller = robot_controller
        self.logger = logger

        # Get model from config or environment
        import os
        self.model = COLLISION_CHECK_MODEL if COLLISION_CHECK_MODEL else os.getenv('ANTHROPIC_MODEL', 'claude-sonnet-4-5-20250929')

        # Load prompts
        with open('prompts/collision_check_system_prompt.txt', 'r') as f:
            self.system_prompt = f.read().strip()
        with open('prompts/collision_check_user_prompt.txt', 'r') as f:
            self.user_prompt_template = f.read().strip()

    def check_step_collision(self, step_command: Dict[str, Any], current_state: Dict[str, Any]) -> CollisionResult:
        """
        Check if executing a step will cause collision.

        Args:
            step_command: Command to check, e.g., {"action": "pick_up", "object": "red_cube"}
            current_state: Current scene state with objects, gripper state, etc.

        Returns:
            CollisionResult with prediction details
        """
        # Build prompt
        user_prompt = self._build_collision_prompt(step_command, current_state)

        # Call LLM
        try:
            messages = [{"role": "user", "content": user_prompt}]

            # Use prompt caching if enabled
            if ENABLE_PROMPT_CACHING:
                response = self.client.messages.create(
                    model=self.model,
                    max_tokens=1024,
                    system=[
                        {
                            "type": "text",
                            "text": self.system_prompt,
                            "cache_control": {"type": "ephemeral"}
                        }
                    ],
                    messages=messages
                )
            else:
                response = self.client.messages.create(
                    model=self.model,
                    max_tokens=1024,
                    system=self.system_prompt,
                    messages=messages
                )

            # Log API call
            self.logger.log_api_call(
                stage="collision_check",
                request={"system": self.system_prompt[:100] + "...", "user": user_prompt[:200] + "..."},
                response=response.content[0].text,
                usage=response.usage
            )

            # Parse response
            result = self._parse_collision_response(response.content[0].text)

            self.logger.log_app_info(f"Collision check: {step_command.get('action')} - Will collide: {result.will_collide}")
            if result.will_collide:
                self.logger.console_warning(f"Collision predicted: {result.reasoning}")

            return result

        except Exception as e:
            self.logger.log_app_error(f"Collision check failed: {e}")
            # Default to safe behavior: assume collision
            return CollisionResult(
                will_collide=True,
                collision_type="unknown",
                risk_level="high",
                affected_objects=[],
                reasoning=f"Collision check error: {str(e)}"
            )

    def _build_collision_prompt(self, step_command: Dict[str, Any], current_state: Dict[str, Any]) -> str:
        """
        Build user prompt for collision checking.

        Args:
            step_command: Command to check
            current_state: Current scene state

        Returns:
            Formatted user prompt string
        """
        # Format objects information
        objects_info = []
        for obj in current_state.get("objects", []):
            obj_str = f"  - {obj['name']}: position {obj['position']}, dimensions {obj['dimensions']}"
            objects_info.append(obj_str)
        objects_str = "\n".join(objects_info)

        # Format gripper information
        gripper = current_state.get("gripper", {})
        gripper_str = (
            f"  - Position: {gripper.get('position')}\n"
            f"  - Orientation: {gripper.get('orientation')}\n"
            f"  - State: {gripper.get('state')}\n"
            f"  - Width: {gripper.get('width')}m"
        )

        # Format holding object
        holding = current_state.get("holding_object")
        holding_str = holding if holding else "none"

        # Format action
        action = step_command.get("action")
        target = step_command.get("object", step_command.get("position", ""))
        action_str = f"{action} {target}"

        # Fill template
        user_prompt = self.user_prompt_template.format(
            objects_info=objects_str,
            gripper_info=gripper_str,
            holding_object=holding_str,
            action_command=action_str
        )

        return user_prompt

    def _parse_collision_response(self, response_text: str) -> CollisionResult:
        """
        Parse LLM response into CollisionResult.

        Args:
            response_text: Raw LLM response

        Returns:
            CollisionResult object
        """
        try:
            # Try to find JSON in response
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1

            if start_idx == -1 or end_idx == 0:
                raise ValueError("No JSON found in response")

            json_str = response_text[start_idx:end_idx]
            data = json.loads(json_str)

            return CollisionResult(
                will_collide=data.get("will_collide", False),
                collision_type=data.get("collision_type", "none"),
                risk_level=data.get("risk_level", "none"),
                affected_objects=data.get("affected_objects", []),
                reasoning=data.get("reasoning", "")
            )

        except Exception as e:
            self.logger.log_app_error(f"Failed to parse collision response: {e}")
            self.logger.app_logger.debug(f"Response text: {response_text}")
            # Default to safe behavior
            return CollisionResult(
                will_collide=True,
                collision_type="unknown",
                risk_level="high",
                affected_objects=[],
                reasoning=f"Parse error: {str(e)}"
            )
