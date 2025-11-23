import pybullet as p
from .config import *


class ObjectManager:
    """
    Manages object loading, tracking, and position queries.
    Maintains a registry of objects and provides methods to interact with them.
    """

    def __init__(self, logger=None):
        """Initialize object manager with empty registry."""
        self.objects = {}
        self.logger = logger

    def load_cube(self, name, position, color, scale=1.0):
        """
        Load a cube URDF and register it with a name.

        Args:
            name: Unique identifier for the object
            position: Base position [x, y, z]
            color: RGBA color [r, g, b, a]
            scale: Global scaling factor (default: 1.0)

        Returns:
            PyBullet object ID
        """
        obj_id = p.loadURDF("cube_small.urdf", basePosition=position, globalScaling=scale)
        p.changeVisualShape(obj_id, GRIPPER_LINK_INDEX, rgbaColor=color)
        self.objects[name] = obj_id
        if self.logger:
            self.logger.log_app_object_loaded(name, position)
        return obj_id

    def get_object_dimensions(self, obj_id):
        """
        Dynamically queries object dimensions from PyBullet based on AABB.

        Args:
            obj_id: PyBullet object ID

        Returns:
            [width, depth, height] list with object dimensions
        """
        aabb_min, aabb_max = p.getAABB(obj_id, -1)

        width = aabb_max[0] - aabb_min[0]
        depth = aabb_max[1] - aabb_min[1]
        height = aabb_max[2] - aabb_min[2]

        return_size = [width, depth, height]
        return return_size

    def get_object_center_position(self, object_name):
        """
        Returns the true geometric center position of an object.

        Calculates the center using AABB (axis-aligned bounding box) to ensure
        accuracy regardless of URDF geometry offsets.

        Args:
            object_name: The name of the object (e.g., 'red_cube')

        Returns:
            [x, y, z] list with the object's center position,
            or None if the object is not found
        """
        if object_name not in self.objects:
            print(f"Error: Object '{object_name}' not found in registry")
            return None

        obj_id = self.objects[object_name]

        # Get AABB bounds and calculate true geometric center
        aabb_min, aabb_max = p.getAABB(obj_id, -1)

        center_pos = [
            (aabb_min[0] + aabb_max[0]) / 2.0,
            (aabb_min[1] + aabb_max[1]) / 2.0,
            (aabb_min[2] + aabb_max[2]) / 2.0
        ]

        return center_pos

    def get_object_id(self, object_name):
        """
        Get the PyBullet object ID for a named object.

        Args:
            object_name: The name of the object

        Returns:
            PyBullet object ID or None if not found
        """
        return self.objects.get(object_name)
