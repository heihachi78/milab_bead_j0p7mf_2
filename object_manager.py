import pybullet as p
from config import *


class ObjectManager:
    """
    Manages object loading, tracking, and position queries.
    Maintains a registry of objects and provides methods to interact with them.
    """

    def __init__(self):
        """Initialize object manager with empty registry."""
        self.objects = {}

    def load_cube(self, name, position, color, scale=OBJECT_SCALE):
        """
        Load a cube URDF and register it with a name.

        Args:
            name: Unique identifier for the object
            position: Base position [x, y, z]
            color: RGBA color [r, g, b, a]
            scale: Global scaling factor

        Returns:
            PyBullet object ID
        """
        obj_id = p.loadURDF("cube_small.urdf", basePosition=position, globalScaling=scale)
        p.changeVisualShape(obj_id, GRIPPER_LINK_INDEX, rgbaColor=color)
        self.objects[name] = obj_id
        print(f"Loaded object '{name}' with ID {obj_id}")
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
        print(f"Object {obj_id} size (AABB): {return_size}")
        return return_size

    def get_object_center_position(self, object_name):
        """
        Returns the center position of an object.

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

        obj_size = self.get_object_dimensions(obj_id)

        base_pos, _ = p.getBasePositionAndOrientation(obj_id)

        center_pos = [
            base_pos[0],
            base_pos[1],
            base_pos[2] + obj_size[2] / 2.0
        ]

        print(f"Object '{object_name}' center position: {center_pos}")

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
