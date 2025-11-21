import pybullet as p
from config import *


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

    def update_debug_lines(self, armId, endEffectorIndex, pos):
        """
        Draw debug lines for robot movement visualization.

        Args:
            armId: PyBullet robot body ID
            endEffectorIndex: Index of the end effector link
            pos: Current target position
        """
        ls = p.getLinkState(armId, endEffectorIndex)

        if self.hasPrevPose:
            p.addUserDebugLine(self.prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.trailDuration)
            p.addUserDebugLine(self.prevPose1, ls[4], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.trailDuration)

        self.prevPose = pos
        self.prevPose1 = ls[4]
        self.hasPrevPose = 1

    def get_state_tuple(self):
        """
        Returns the current state as a tuple for compatibility with existing code.

        Returns:
            Tuple of (t, hasPrevPose, prevPose, prevPose1)
        """
        return (self.t, self.hasPrevPose, self.prevPose, self.prevPose1)
