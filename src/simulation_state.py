"""
Simulation state management.

Tracks simulation time and debug visualization state for the PyBullet
robotics simulation.
"""

import pybullet as p
from .config import *


class SimulationState:
    """
    Manages simulation state including time tracking and debug visualization.
    Handles state propagation through operations.
    """

    def __init__(self):
        """Initialize simulation state with default values."""
        self.t = INITIAL_TIME
        self.prevPose = INITIAL_PREV_POSE.copy()
        self.prevPose1 = INITIAL_PREV_POSE.copy()
        self.hasPrevPose = INITIAL_HAS_PREV_POSE
        self.trailDuration = TRAIL_DURATION

    def step_time(self):
        """Advance simulation time by one time step."""
        self.t += TIME_STEP
