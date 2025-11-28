"""
Multi-camera management for scene visualization.

Provides functionality for capturing images from multiple camera angles
and compositing them into panorama images for LLM analysis and debugging.
"""

import pybullet as p
import numpy as np
from PIL import Image
import os
from .config import *


class CameraManager:
    """
    Manages multi-camera setup for capturing images from different angles
    and creating panorama images.
    """

    def __init__(self, logger=None):
        """
        Initialize camera manager with configuration from config.py.

        Sets up camera parameters and creates the images output folder.

        Args:
            logger: SimulationLogger instance for logging (optional)
        """
        self.logger = logger
        self.target_position = CAMERA_TARGET_POSITION
        self.distance = CAMERA_DISTANCE
        self.width = CAMERA_IMAGE_WIDTH
        self.height = CAMERA_IMAGE_HEIGHT
        self.fov = CAMERA_FOV
        self.near = CAMERA_NEAR_PLANE
        self.far = CAMERA_FAR_PLANE
        self.yaw_angles = CAMERA_YAW_ANGLES
        self.pitch_angles = CAMERA_PITCH_ANGLES
        self.images_folder = IMAGES_FOLDER

        # Create images folder if it doesn't exist
        os.makedirs(self.images_folder, exist_ok=True)

        # Counter for panorama images
        self.panorama_counter = 0

    def capture_image(self, yaw, pitch):
        """
        Capture a single image from a specific camera angle.

        Args:
            yaw: Horizontal angle (azimuth) in degrees
            pitch: Vertical angle (elevation) in degrees

        Returns:
            numpy array: RGB image data (height x width x 3)
        """
        # Compute view matrix
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.target_position,
            distance=self.distance,
            yaw=yaw,
            pitch=pitch,
            roll=0,
            upAxisIndex=2
        )

        # Compute projection matrix
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=self.width / self.height,
            nearVal=self.near,
            farVal=self.far
        )

        # Capture image
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # Convert to numpy array and remove alpha channel
        rgb_array = np.array(rgb_img, dtype=np.uint8)
        rgb_array = rgb_array.reshape((self.height, self.width, 4))[:, :, :3]

        return rgb_array

    def capture_multi_camera(self):
        """
        Capture images from all 5 camera positions.

        Returns:
            list: List of numpy arrays containing RGB images from each angle
        """
        images = []
        directions = ['front', 'right', 'back', 'left', 'top']

        for i, (yaw, pitch) in enumerate(zip(self.yaw_angles, self.pitch_angles)):
            img = self.capture_image(yaw, pitch)
            images.append(img)

        return images

    def create_panorama(self, images):
        """
        Create a panorama image from the captured images.
        Layout: [front, right, back, left] in a row, with top view centered below.

        Args:
            images: List of 5 numpy arrays (front, right, back, left, top)

        Returns:
            PIL.Image: Panorama image
        """
        # Convert numpy arrays to PIL images
        pil_images = [Image.fromarray(img) for img in images]

        # Create panorama layout: 4 side views in a row, top view centered below
        # Top row: 4 side views (front, right, back, left)
        top_row_width = self.width * 4
        top_row_height = self.height

        # Bottom row: top view centered
        bottom_row_width = top_row_width
        bottom_row_height = self.height

        # Create blank canvas
        panorama_width = top_row_width
        panorama_height = top_row_height + bottom_row_height
        panorama = Image.new('RGB', (panorama_width, panorama_height), color=(50, 50, 50))

        # Paste side views in top row
        for i in range(4):
            x_offset = i * self.width
            panorama.paste(pil_images[i], (x_offset, 0))

        # Paste top view centered in bottom row
        top_view_x_offset = (panorama_width - self.width) // 2
        panorama.paste(pil_images[4], (top_view_x_offset, top_row_height))

        return panorama

    def save_panorama(self, panorama, operation_name):
        """
        Save the panorama image to the images folder.

        Args:
            panorama: PIL.Image object
            operation_name: Name of the operation (e.g., 'pickup', 'place')

        Returns:
            str: Path to saved image
        """
        # Determine format and extension from config
        format_type = PANORAMA_FORMAT if hasattr(globals(), 'PANORAMA_FORMAT') else 'JPEG'
        extension = 'jpg' if format_type == 'JPEG' else 'png'

        filename = f"panorama_{self.panorama_counter:03d}_{operation_name}.{extension}"
        filepath = os.path.join(self.images_folder, filename)

        # Save with format-specific options
        if format_type == 'JPEG':
            quality = PANORAMA_QUALITY if hasattr(globals(), 'PANORAMA_QUALITY') else 75
            panorama.save(filepath, format='JPEG', quality=quality, optimize=True)
        else:
            panorama.save(filepath, format='PNG')

        if self.logger:
            self.logger.log_app_camera_capture("panorama", filename)

        self.panorama_counter += 1
        return filepath

    def capture_and_save_panorama(self, operation_name):
        """
        Capture images from all cameras, create panorama, and save.

        Args:
            operation_name: Name of the operation (e.g., 'pickup', 'place')

        Returns:
            str: Path to saved panorama image
        """
        # Capture images from all angles
        images = self.capture_multi_camera()

        # Create panorama
        panorama = self.create_panorama(images)

        # Save panorama
        filepath = self.save_panorama(panorama, operation_name)

        return filepath
