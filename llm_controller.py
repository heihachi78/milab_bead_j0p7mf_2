import os
import json
import base64
from pathlib import Path
from typing import Dict, List, Any
from dotenv import load_dotenv
from anthropic import Anthropic


class LLMController:
    """Controller for LLM-based robot task planning and execution."""

    def __init__(self, object_manager, robot_controller):
        """
        Initialize the LLM controller.

        Args:
            object_manager: ObjectManager instance for querying object positions
            robot_controller: RobotController instance for executing commands
        """
        self.object_manager = object_manager
        self.robot_controller = robot_controller

        # Load environment variables
        load_dotenv()
        self.api_key = os.getenv('ANTHROPIC_API_KEY')
        if not self.api_key:
            raise ValueError("ANTHROPIC_API_KEY not found in .env file")

        # Initialize Anthropic client
        self.client = Anthropic(api_key=self.api_key)
        self.model = os.getenv('ANTHROPIC_MODEL', 'claude-3-5-sonnet-20241022')

        # Load system prompt template and task description
        self.system_prompt_template = self._load_file('system_prompt.txt')
        self.task_description = self._load_file('llm_task_init.txt')

    def _load_file(self, filename: str) -> str:
        """Load text file content."""
        filepath = Path(__file__).parent / filename
        with open(filepath, 'r') as f:
            return f.read().strip()

    def _encode_image(self, image_path: str) -> str:
        """Encode image to base64."""
        with open(image_path, 'rb') as f:
            return base64.standard_b64encode(f.read()).decode('utf-8')

    def _build_system_prompt(self) -> str:
        """Build the system prompt with current object information."""
        # Get all objects
        objects = self.object_manager.objects

        # Build objects list
        objects_list = "- " + "\n- ".join([f"{name}" for name in objects.keys()])

        # Build objects info with positions and dimensions
        objects_info = []
        for name, obj_id in objects.items():
            pos = self.object_manager.get_object_center_position(name)
            dims = self.object_manager.get_object_dimensions(obj_id)
            objects_info.append(
                f"- {name}:\n"
                f"  - Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] meters\n"
                f"  - Dimensions: [{dims[0]:.3f}, {dims[1]:.3f}, {dims[2]:.3f}] meters (width, depth, height)"
            )
        objects_info_str = "\n".join(objects_info)

        # Fill in the template
        system_prompt = self.system_prompt_template.format(
            OBJECTS_LIST=objects_list,
            OBJECTS_INFO=objects_info_str,
            TASK_DESCRIPTION=self.task_description
        )

        return system_prompt

    def _find_latest_panorama(self, pattern: str = "panorama_*_initial_stabilized_*.png") -> str:
        """Find the most recent panorama image matching the pattern."""
        images_dir = Path(__file__).parent / "images"
        if not images_dir.exists():
            raise FileNotFoundError(f"Images directory not found: {images_dir}")

        # Find all matching panorama files
        import glob
        matching_files = glob.glob(str(images_dir / pattern))

        if not matching_files:
            raise FileNotFoundError(f"No panorama images found matching pattern: {pattern}")

        # Sort by modification time and get the most recent
        latest_file = max(matching_files, key=os.path.getmtime)
        return latest_file

    def generate_plan(self, panorama_path: str = None) -> Dict[str, Any]:
        """
        Generate a task plan using the LLM.

        Args:
            panorama_path: Path to panorama image. If None, finds the latest initial panorama.

        Returns:
            Dictionary containing the parsed JSON response from the LLM
        """
        print(f"=== LLM TASK PLANNING START ===")

        # Find panorama if not provided
        if panorama_path is None:
            panorama_path = self._find_latest_panorama()

        print(f"Panorama: {panorama_path}")
        print(f"Task: {self.task_description}")

        # Build system prompt with current object state
        system_prompt = self._build_system_prompt()

        # Log the full system prompt
        print(f"\n--- SYSTEM PROMPT START ---")
        print(system_prompt)
        print(f"--- SYSTEM PROMPT END ---\n")

        # Encode image
        image_data = self._encode_image(panorama_path)

        # Prepare user message
        user_message_text = "Analyze the scene in the panorama image and generate the command sequence to complete the task."

        print(f"--- USER MESSAGE ---")
        print(f"[Panorama Image: {panorama_path}]")
        print(user_message_text)
        print(f"--- END USER MESSAGE ---\n")

        # Call Anthropic API
        print(f"Calling Anthropic API...")
        print(f"Model: {self.model}")

        try:
            response = self.client.messages.create(
                model=self.model,
                max_tokens=2048,
                system=system_prompt,
                messages=[
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
                                "text": user_message_text
                            }
                        ],
                    }
                ],
            )

            # Extract text response
            response_text = response.content[0].text

            print(f"\n--- LLM RESPONSE START ---")
            print(response_text)
            print(f"--- LLM RESPONSE END ---\n")

            # Parse JSON from response
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

            plan = json.loads(json_str)

            print(f"Parsed plan:")
            print(json.dumps(plan, indent=2))
            print(f"=== LLM TASK PLANNING END ===\n")

            return plan

        except json.JSONDecodeError as e:
            print(f"\nERROR: Failed to parse JSON response: {e}")
            print(f"Response text: {response_text}")
            raise
        except Exception as e:
            print(f"\nERROR: API call failed: {e}")
            raise

    def execute_plan(self, plan: Dict[str, Any]) -> None:
        """
        Execute a command plan generated by the LLM.

        Args:
            plan: Dictionary containing 'commands' list with action/parameter pairs
        """
        if 'commands' not in plan:
            raise ValueError("Plan must contain 'commands' key")

        commands = plan['commands']

        print(f"=== EXECUTE LLM PLAN START === Commands: {len(commands)}")

        for i, command in enumerate(commands, 1):
            action = command.get('action')

            if action == 'pick_up':
                object_name = command.get('object')
                if not object_name:
                    raise ValueError(f"Command {i}: 'pick_up' requires 'object' parameter")

                print(f"\n--- Command {i}/{len(commands)}: pick_up('{object_name}') ---")
                self.robot_controller.pick_up(object_name)

            elif action == 'place':
                position = command.get('position')
                if not position or len(position) != 3:
                    raise ValueError(f"Command {i}: 'place' requires 'position' parameter as [x, y, z]")

                print(f"\n--- Command {i}/{len(commands)}: place({position}) ---")
                self.robot_controller.place(position)

            else:
                raise ValueError(f"Command {i}: Unknown action '{action}'")

        print(f"=== EXECUTE LLM PLAN END ===")
