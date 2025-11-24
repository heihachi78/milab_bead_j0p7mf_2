"""
Vision analysis module for gripper rotation guidance and approach feasibility.

Uses a vision-language model (BLIP or BLIP2) to analyze scene panoramas and provide:
1. Whether the gripper needs to be rotated from its default downward orientation
2. Whether the gripper can clearly approach each object from above without obstruction

Returns simple yes/no guidance per object - no geometric calculations.
Model outputs are logged to JSON for inspection.
"""

import os
import json
from datetime import datetime
from typing import Dict, List, Tuple, Optional
import numpy as np
from PIL import Image

# Set tokenizers parallelism before importing transformers to avoid fork warnings
os.environ["TOKENIZERS_PARALLELISM"] = "false"

from .config import (
    VISION_MODEL_NAME,
    VISION_MODEL_DEVICE,
    VISION_ANALYSIS_LOG_DIR,
    VISION_ANALYSIS_CACHE_DIR,
    VISION_APPROACH_ANALYSIS_ENABLED,
)


class VisionAnalyzer:
    """
    Analyzes scene panoramas to determine gripper orientations and approach feasibility.

    Provides vision-based analysis for:
    - Gripper rotation requirements for grasping objects
    - Whether objects can be approached from above without obstruction
    """

    def __init__(self, logger=None):
        """
        Initialize the vision analyzer with HuggingFace model.

        Args:
            logger: Optional logger instance for logging
        """
        self.logger = logger
        self.model = None
        self.processor = None
        self.device = VISION_MODEL_DEVICE

        # Initialize model lazily (only when needed)
        self._model_initialized = False

    def _initialize_model(self):
        """Lazy initialization of the HuggingFace model."""
        if self._model_initialized:
            return

        try:
            if self.logger:
                self.logger.log_app_info(f"Initializing vision model: {VISION_MODEL_NAME}")

            import torch

            # Detect if using BLIP or BLIP2
            if "blip2" in VISION_MODEL_NAME.lower():
                from transformers import Blip2Processor, Blip2ForConditionalGeneration
                self.processor = Blip2Processor.from_pretrained(
                    VISION_MODEL_NAME,
                    cache_dir=VISION_ANALYSIS_CACHE_DIR
                )
                # Load BLIP2 model
                self.model = Blip2ForConditionalGeneration.from_pretrained(
                    VISION_MODEL_NAME,
                    cache_dir=VISION_ANALYSIS_CACHE_DIR,
                    torch_dtype=torch.float32 if self.device == "cpu" else torch.float16
                )
            else:
                # Use BLIP (faster, smaller model)
                from transformers import BlipProcessor, BlipForQuestionAnswering
                self.processor = BlipProcessor.from_pretrained(
                    VISION_MODEL_NAME,
                    cache_dir=VISION_ANALYSIS_CACHE_DIR
                )
                self.model = BlipForQuestionAnswering.from_pretrained(
                    VISION_MODEL_NAME,
                    cache_dir=VISION_ANALYSIS_CACHE_DIR
                )

            # Move model to device
            self.model.to(self.device)
            self.model.eval()

            self._model_initialized = True

            if self.logger:
                self.logger.log_app_info(f"Vision model initialized on {self.device}")

        except Exception as e:
            if self.logger:
                self.logger.console_error(f"Failed to initialize vision model: {str(e)}")
            raise

    def analyze_scene(
        self,
        panorama_path: str,
        object_info: Dict[str, Dict],
        scene_name: str = "unknown"
    ) -> Dict:
        """
        Main entry point for scene analysis.

        Args:
            panorama_path: Path to the panorama image
            object_info: Dictionary containing object positions and dimensions
            scene_name: Name of the scene being analyzed

        Returns:
            Dictionary containing analysis results for all objects
        """
        start_time = datetime.now()

        if self.logger:
            self.logger.log_app_info("Starting vision analysis...")

        try:
            # Initialize model if not already done
            self._initialize_model()

            # Load panorama image
            panorama_image = Image.open(panorama_path)

            # Analyze each object
            analysis_results = {}
            for obj_name, obj_data in object_info.items():
                if self.logger:
                    self.logger.log_app_info(f"  Analyzing object: {obj_name}")

                obj_analysis = self._analyze_object(
                    panorama_image,
                    obj_name,
                    obj_data,
                    object_info
                )
                analysis_results[obj_name] = obj_analysis

            # Calculate duration
            duration = (datetime.now() - start_time).total_seconds()

            # Create full analysis output
            full_analysis = {
                "timestamp": datetime.now().isoformat(),
                "scene": scene_name,
                "panorama_path": panorama_path,
                "analysis_duration_seconds": duration,
                "model_name": VISION_MODEL_NAME,
                "device": self.device,
                "analysis": analysis_results
            }

            # Save to log file
            log_path = self._save_analysis_to_log(full_analysis)

            if self.logger:
                self.logger.log_app_info(f"Vision analysis completed in {duration:.2f}s")

            return full_analysis

        except Exception as e:
            if self.logger:
                self.logger.console_error(f"Vision analysis failed: {str(e)}")

            # Return empty analysis on failure
            return {
                "timestamp": datetime.now().isoformat(),
                "scene": scene_name,
                "panorama_path": panorama_path,
                "error": str(e),
                "analysis": {}
            }

    def _analyze_object(
        self,
        panorama_image: Image.Image,
        obj_name: str,
        obj_data: Dict,
        all_objects: Dict[str, Dict]
    ) -> Dict:
        """
        Analyze a single object using the vision model.

        Args:
            panorama_image: PIL Image of the panorama
            obj_name: Name of the object to analyze
            obj_data: Dictionary with position and dimensions
            all_objects: All objects in the scene

        Returns:
            Dictionary with vision model outputs only
        """
        position = obj_data.get("position", [0, 0, 0])
        dimensions = obj_data.get("dimensions", [0.05, 0.05, 0.05])

        # Analyze gripper rotation requirement
        vision_analysis = self._analyze_object_grippability(
            panorama_image,
            obj_name,
            position,
            dimensions
        )

        # Analyze approach feasibility if enabled
        if VISION_APPROACH_ANALYSIS_ENABLED:
            approach_analysis = self._analyze_object_approach_feasibility(
                panorama_image,
                obj_name,
                position,
                dimensions
            )
            # Merge approach analysis into results
            vision_analysis.update(approach_analysis)

        return vision_analysis

    def _analyze_object_grippability(
        self,
        panorama_image: Image.Image,
        obj_name: str,
        position: List[float],
        dimensions: List[float]
    ) -> Dict:
        """
        Use vision model to determine if gripper rotation is needed.

        Args:
            panorama_image: PIL Image of the panorama
            obj_name: Name of the object
            position: [x, y, z] position in world coordinates
            dimensions: [width, depth, height] in meters

        Returns:
            Dictionary with 'needs_rotation' (yes/no)
        """
        import torch

        # Load prompt template
        prompt_path = os.path.join("prompts", "vision_gripper_rotation.txt")
        with open(prompt_path, 'r') as f:
            prompt_template = f.read().strip()

        # Fill in the object name
        prompt = prompt_template.format(object_name=obj_name)

        # Process image and prompt
        inputs = self.processor(
            images=panorama_image,
            text=prompt,
            return_tensors="pt"
        ).to(self.device)

        # Generate response with parameters optimized for simple yes/no answers
        with torch.no_grad():
            generated_ids = self.model.generate(
                **inputs,
                max_new_tokens=10,  # Short responses
                num_beams=3,  # Fewer beams for simple answers
                do_sample=False,  # Use beam search, not sampling
                early_stopping=True
            )

        # Decode the response
        response = self.processor.decode(generated_ids[0], skip_special_tokens=True)
        response = response.strip()

        # Parse response into yes/no
        # Note: Question asks "Does the object have another cube directly touching its side?"
        # So YES means cube touching side (needs rotation), NO means no side contact (no rotation)
        response_lower = response.lower()

        # Determine yes or no (direct logic)
        if response_lower.startswith('yes'):
            needs_rotation = "yes"  # Yes cube touching side = needs rotation
        elif response_lower.startswith('no'):
            needs_rotation = "no"  # No cube touching side = no rotation needed
        else:
            # If model doesn't start with yes/no, try to extract it
            needs_rotation = "yes" if "yes" in response_lower else "no"

        result = {
            "needs_rotation": needs_rotation
        }

        return result

    def _analyze_object_approach_feasibility(
        self,
        panorama_image: Image.Image,
        obj_name: str,
        position: List[float],
        dimensions: List[float]
    ) -> Dict:
        """
        Use vision model to determine if gripper can approach from above.

        Args:
            panorama_image: PIL Image of the panorama
            obj_name: Name of the object
            position: [x, y, z] position in world coordinates
            dimensions: [width, depth, height] in meters

        Returns:
            Dictionary with 'can_approach_from_above' (yes/no)
        """
        import torch

        # Load prompt template
        prompt_path = os.path.join("prompts", "vision_gripper_approach.txt")
        with open(prompt_path, 'r') as f:
            prompt_template = f.read().strip()

        # Fill in the object name
        prompt = prompt_template.format(object_name=obj_name)

        # Process image and prompt
        inputs = self.processor(
            images=panorama_image,
            text=prompt,
            return_tensors="pt"
        ).to(self.device)

        # Generate response with parameters optimized for simple yes/no answers
        with torch.no_grad():
            generated_ids = self.model.generate(
                **inputs,
                max_new_tokens=10,  # Short responses
                num_beams=3,  # Fewer beams for simple answers
                do_sample=False,  # Use beam search, not sampling
                early_stopping=True
            )

        # Decode the response
        response = self.processor.decode(generated_ids[0], skip_special_tokens=True)
        response = response.strip()

        # Parse response into yes/no
        # Note: Question asks "Is the object at the very top of the stack?"
        # So YES means clear (can approach), NO means blocked (can't approach)
        response_lower = response.lower()

        # Determine yes or no (direct logic)
        if response_lower.startswith('yes'):
            can_approach = "yes"  # Yes it's at the top = can approach
        elif response_lower.startswith('no'):
            can_approach = "no"  # No it's not at the top = can't approach
        else:
            # If model doesn't start with yes/no, try to extract it
            can_approach = "yes" if "yes" in response_lower else "no"

        result = {
            "can_approach_from_above": can_approach
        }

        return result

    def _save_analysis_to_log(self, analysis: Dict) -> str:
        """
        Save analysis results to a JSON log file using the logger.

        Args:
            analysis: Analysis results dictionary

        Returns:
            Path to the saved log file
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"vision_analysis_{timestamp}.json"
        filepath = os.path.join(VISION_ANALYSIS_LOG_DIR, filename)

        if self.logger:
            self.logger.log_vision_analysis(analysis, filepath)
        else:
            # Fallback if no logger available
            with open(filepath, 'w') as f:
                json.dump(analysis, f, indent=2)

        return filepath
