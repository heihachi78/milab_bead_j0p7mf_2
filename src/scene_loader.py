"""
Scene loader for YAML-based scene configuration files.

This module handles loading, parsing, and validating scene configuration files
that define objects and tasks for the PyBullet robotics simulation.
"""

import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import List, Dict, Any


@dataclass
class ObjectConfig:
    """Configuration for a single object in the scene."""
    name: str
    type: str
    position: List[float]
    color: List[float]
    scale: float = 1.0


@dataclass
class TaskConfig:
    """Configuration for the task to be performed."""
    description: str


@dataclass
class SceneMetadata:
    """Metadata about the scene."""
    name: str
    description: str


@dataclass
class SceneConfig:
    """Complete scene configuration including metadata, objects, and task."""
    metadata: SceneMetadata
    objects: List[ObjectConfig]
    task: TaskConfig

    def get_object_names(self) -> List[str]:
        """Get list of all object names in the scene."""
        return [obj.name for obj in self.objects]


class SceneLoadError(Exception):
    """Exception raised when scene loading or validation fails."""
    pass


def load_scene(scene_name: str, scenes_folder: str = "scenes") -> SceneConfig:
    """
    Load a scene configuration from a YAML file.

    Args:
        scene_name: Name of the scene file (with or without .yaml extension)
        scenes_folder: Path to the scenes folder (default: "scenes")

    Returns:
        SceneConfig object containing the parsed scene configuration

    Raises:
        SceneLoadError: If the scene file cannot be found or parsed
    """
    # Strip .yaml extension if provided
    if scene_name.endswith('.yaml'):
        scene_name = scene_name[:-5]

    # Construct the scene file path
    scenes_path = Path(scenes_folder)
    scene_file = scenes_path / f"{scene_name}.yaml"

    # Check if file exists
    if not scene_file.exists():
        available_scenes = list_available_scenes(scenes_folder)
        raise SceneLoadError(
            f"Scene file not found: {scene_file}\n"
            f"Available scenes: {', '.join(available_scenes)}"
        )

    # Load and parse YAML
    try:
        with open(scene_file, 'r') as f:
            scene_data = yaml.safe_load(f)
    except yaml.YAMLError as e:
        raise SceneLoadError(f"Failed to parse YAML file {scene_file}: {e}")
    except Exception as e:
        raise SceneLoadError(f"Failed to read scene file {scene_file}: {e}")

    # Validate and parse the scene data
    try:
        scene_config = _parse_scene_data(scene_data)
    except Exception as e:
        raise SceneLoadError(f"Failed to validate scene configuration: {e}")

    return scene_config


def _parse_scene_data(data: Dict[str, Any]) -> SceneConfig:
    """
    Parse and validate scene data from a dictionary.

    Args:
        data: Dictionary containing scene data from YAML

    Returns:
        SceneConfig object

    Raises:
        ValueError: If required fields are missing or invalid
    """
    # Parse metadata
    if 'metadata' not in data:
        raise ValueError("Missing required 'metadata' section")

    metadata_data = data['metadata']
    metadata = SceneMetadata(
        name=metadata_data.get('name', 'Unnamed Scene'),
        description=metadata_data.get('description', '')
    )

    # Parse objects
    if 'objects' not in data:
        raise ValueError("Missing required 'objects' section")

    if not isinstance(data['objects'], list) or len(data['objects']) == 0:
        raise ValueError("'objects' must be a non-empty list")

    objects = []
    for i, obj_data in enumerate(data['objects']):
        try:
            obj = _parse_object(obj_data)
            objects.append(obj)
        except Exception as e:
            raise ValueError(f"Error parsing object {i}: {e}")

    # Check for duplicate object names
    object_names = [obj.name for obj in objects]
    if len(object_names) != len(set(object_names)):
        duplicates = [name for name in object_names if object_names.count(name) > 1]
        raise ValueError(f"Duplicate object names found: {set(duplicates)}")

    # Parse task
    if 'task' not in data:
        raise ValueError("Missing required 'task' section")

    task_data = data['task']
    if 'description' not in task_data:
        raise ValueError("Task section must contain 'description' field")

    task = TaskConfig(description=task_data['description'])

    return SceneConfig(metadata=metadata, objects=objects, task=task)


def _parse_object(obj_data: Dict[str, Any]) -> ObjectConfig:
    """
    Parse object configuration from dictionary.

    Args:
        obj_data: Dictionary containing object data

    Returns:
        ObjectConfig object

    Raises:
        ValueError: If required fields are missing or invalid
    """
    # Required fields
    if 'name' not in obj_data:
        raise ValueError("Object missing required 'name' field")
    if 'type' not in obj_data:
        raise ValueError(f"Object '{obj_data['name']}' missing required 'type' field")
    if 'position' not in obj_data:
        raise ValueError(f"Object '{obj_data['name']}' missing required 'position' field")
    if 'color' not in obj_data:
        raise ValueError(f"Object '{obj_data['name']}' missing required 'color' field")

    # Validate position
    position = obj_data['position']
    if not isinstance(position, list) or len(position) != 3:
        raise ValueError(f"Object '{obj_data['name']}' position must be [x, y, z]")
    if not all(isinstance(v, (int, float)) for v in position):
        raise ValueError(f"Object '{obj_data['name']}' position values must be numbers")

    # Validate color
    color = obj_data['color']
    if not isinstance(color, list) or len(color) != 4:
        raise ValueError(f"Object '{obj_data['name']}' color must be [r, g, b, a]")
    if not all(isinstance(v, (int, float)) for v in color):
        raise ValueError(f"Object '{obj_data['name']}' color values must be numbers")
    if not all(0 <= v <= 1 for v in color):
        raise ValueError(f"Object '{obj_data['name']}' color values must be in range [0, 1]")

    # Optional scale field
    scale = obj_data.get('scale', 1.0)
    if not isinstance(scale, (int, float)) or scale <= 0:
        raise ValueError(f"Object '{obj_data['name']}' scale must be a positive number")

    return ObjectConfig(
        name=obj_data['name'],
        type=obj_data['type'],
        position=position,
        color=color,
        scale=scale
    )


def list_available_scenes(scenes_folder: str = "scenes") -> List[str]:
    """
    List all available scene files in the scenes folder.

    Args:
        scenes_folder: Path to the scenes folder

    Returns:
        List of scene names (without .yaml extension)
    """
    scenes_path = Path(scenes_folder)
    if not scenes_path.exists():
        return []

    scene_files = scenes_path.glob("*.yaml")
    return sorted([f.stem for f in scene_files])
