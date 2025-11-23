import numpy as np
import pybullet as p
import pybullet_data
import math
from datetime import datetime
from .config import *


class RobotController:
    """
    Controls robot movements, gripper operations, and high-level pick-and-place tasks.
    Encapsulates all robot control logic including IK, motion planning, and gripper control.
    """

    @staticmethod
    def initialize_pybullet(logger=None, mode='auto'):
        """
        Initialize PyBullet connection and load basic simulation environment.

        Args:
            logger: SimulationLogger instance for logging (optional)
            mode: Connection mode - 'auto' (try shared then GUI), 'shared_only' (try shared then direct),
                  'gui', or 'direct' (default: 'auto')
            configure_gui: If True, disable shadows, GUI, and preview windows (default: False)

        Returns:
            tuple: (armId, connection_mode) where connection_mode is 'shared', 'direct', or 'gui'
        """
        # Connect to PyBullet
        connection_mode = 'direct'

        if mode == 'shared_only':
            clid = p.connect(p.SHARED_MEMORY)
            if clid < 0:
                if logger:
                    logger.console_info("No PyBullet GUI server found, using headless mode")
                p.connect(p.DIRECT)
                connection_mode = 'direct'
            else:
                if logger:
                    logger.console_info("Connected to PyBullet GUI server")
                connection_mode = 'shared'
        elif mode == 'gui':
            # Direct GUI connection
            p.connect(p.GUI)
            connection_mode = 'gui'
            if logger:
                logger.console_info("Started PyBullet GUI")
        elif mode == 'direct':
            # Direct headless connection
            p.connect(p.DIRECT)
            connection_mode = 'direct'
            if logger:
                logger.console_info("Using headless mode")
        else:
            raise ValueError(f"Invalid mode: {mode}. Must be 'shared_only', 'gui', or 'direct'")

        # Configure debug visualizer if requested (only for GUI/shared modes, not DIRECT)
        # Note: configureDebugVisualizer can hang on macOS in DIRECT mode
        if connection_mode != 'direct':
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        if logger:
            logger.app_logger.info("PyBullet connected successfully")

        # Try to find existing robot in the simulation (if connecting to shared memory)
        armId = None
        if connection_mode == 'shared':
            for i in range(p.getNumBodies()):
                body_info = p.getBodyInfo(i)
                body_name = body_info[0].decode('utf-8')
                if 'panda' in body_name.lower() or i == 1:  # Robot is typically body ID 1
                    armId = i
                    break

        # If no robot found, create new simulation environment
        if armId is None:
            if logger and connection_mode == 'shared':
                logger.console_info("No existing simulation found, creating new one...")

            # Load plane
            p.loadURDF("plane.urdf", PLANE_POSITION)
            if logger:
                logger.app_logger.info(f"Loaded plane at {PLANE_POSITION}")

            # Load robot
            armId = p.loadURDF("franka_panda/panda.urdf", ROBOT_BASE_POSITION, useFixedBase=True)
            p.resetBasePositionAndOrientation(armId, ROBOT_BASE_POSITION, ROBOT_BASE_ORIENTATION)
            if logger:
                logger.app_logger.info(f"Loaded Franka Panda robot")

            # Set physics parameters
            p.setGravity(0, 0, GRAVITY)
            if logger:
                logger.app_logger.info(f"Gravity set to {GRAVITY}")

            p.setRealTimeSimulation(USE_REAL_TIME_SIMULATION)
            if logger:
                logger.app_logger.info(f"Real-time simulation: {USE_REAL_TIME_SIMULATION}")

        # Log robot info
        numJoints = p.getNumJoints(armId)
        if logger:
            logger.app_logger.info(f"Using Franka Panda robot with {numJoints} joints (ID: {armId})")
            logger.console_info(f"Robot ready: {numJoints} joints")

        return armId, connection_mode

    def __init__(self, armId, endEffectorIndex, simulation_state, object_manager, camera_manager=None, logger=None):
        """
        Initialize robot controller.

        Args:
            armId: PyBullet robot body ID
            endEffectorIndex: Index of the end effector link
            simulation_state: SimulationState instance for state tracking
            object_manager: ObjectManager instance for object queries
            camera_manager: CameraManager instance for panorama capture (optional)
            logger: SimulationLogger instance for logging (optional)
        """
        self.armId = armId
        self.endEffectorIndex = endEffectorIndex
        self.simulation_state = simulation_state
        self.object_manager = object_manager
        self.camera_manager = camera_manager
        self.logger = logger

        # Joint configuration
        self.ll = LOWER_LIMITS
        self.ul = UPPER_LIMITS
        self.jr = JOINT_RANGES
        self.rp = REST_POSES
        self.jd = JOINT_DAMPING

        # Control parameters
        self.useNullSpace = USE_NULL_SPACE
        self.useOrientation = USE_ORIENTATION
        self.useSimulation = USE_SIMULATION
        self.useRealTimeSimulation = USE_REAL_TIME_SIMULATION
        self.ikSolver = IK_SOLVER

        # Gripper state tracking
        self.current_gripper_pos = GRIPPER_CLOSED_POSITION

        # Gripper orientation (default: pointing down)
        self.orn = p.getQuaternionFromEuler([0.0, -math.pi, 0.0])

        # Configure gripper for better grip strength
        self._configure_gripper()

    def _configure_gripper(self):
        """
        Configure gripper dynamics and constraints for improved grip strength.
        Increases friction and creates a gear constraint to mechanically lock fingers together.
        """
        # Increase gripper finger friction for better grip
        p.changeDynamics(self.armId, FINGER_JOINT1_INDEX, lateralFriction=2.0)
        p.changeDynamics(self.armId, FINGER_JOINT2_INDEX, lateralFriction=2.0)
        if self.logger:
            self.logger.app_logger.info("Configured gripper friction: lateralFriction=2.0")

        # Create gear constraint to mechanically lock gripper fingers together
        # This prevents inertial forces from breaking the grip during fast movements
        # Gear ratio of -1 means fingers move symmetrically in opposite directions
        self.gripper_constraint = p.createConstraint(
            parentBodyUniqueId=self.armId,
            parentLinkIndex=FINGER_JOINT1_INDEX,
            childBodyUniqueId=self.armId,
            childLinkIndex=FINGER_JOINT2_INDEX,
            jointType=p.JOINT_GEAR,
            jointAxis=[1, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0]
        )
        p.changeConstraint(self.gripper_constraint, gearRatio=-1, erp=0.1, maxForce=50)
        if self.logger:
            self.logger.app_logger.info("Created gear constraint between gripper fingers (gearRatio=-1, maxForce=50)")

    def get_end_effector_position(self):
        """
        Get the current position of the end effector.

        Returns:
            list: Current end effector position [x, y, z]
        """
        ls = p.getLinkState(self.armId, self.endEffectorIndex)
        return list(ls[0])

    def reset_orientation(self):
        """
        Reset gripper orientation to default down-pointing orientation.
        Orientation: [roll=0.0, pitch=-π, yaw=0.0]
        Rotates the gripper in place without moving the arm position.
        """
        self.orn = p.getQuaternionFromEuler([0.0, -math.pi, 0.0])
        if self.logger:
            self.logger.app_logger.info("Reset gripper orientation to default: [0.0, -π, 0.0]")

        # Rotate in place by moving to current position with new orientation
        # Use move_to_target (not smooth) with a looser threshold to allow the robot
        # to adjust its joint configuration for the new orientation
        current_pos = self.get_end_effector_position()
        self.move_to_target(current_pos, THRESHOLD_CLOSE_TARGET)

    def rotate_orientation_90(self):
        """
        Set gripper orientation to 90 degrees rotated (around yaw axis).
        Orientation: [roll=0.0, pitch=-π, yaw=-π/2]
        This allows the gripper to approach objects from a perpendicular direction.
        Rotates the gripper in place without moving the arm position.
        """
        self.orn = p.getQuaternionFromEuler([0.0, -math.pi, -math.pi / 2])
        if self.logger:
            self.logger.app_logger.info("Set gripper orientation to 90° rotated: [0.0, -π, -π/2]")

        # Rotate in place by moving to current position with new orientation
        # Use move_to_target (not smooth) with a looser threshold to allow the robot
        # to adjust its joint configuration for the new orientation
        current_pos = self.get_end_effector_position()
        self.move_to_target(current_pos, THRESHOLD_CLOSE_TARGET)

    def stabilize(self):
        """
        Reset robot to rest pose and stabilize with motors.

        Resets all arm joints to rest positions and applies motor control
        to stabilize the robot. Also initializes gripper in closed position.
        Runs physics simulation for STABILIZATION_STEPS steps.
        """
        if self.logger:
            self.logger.log_robot_stabilize(STABILIZATION_STEPS)
        for i in range(NUM_ARM_JOINTS):
            p.resetJointState(self.armId, i, self.rp[i])

        for i in range(NUM_ARM_JOINTS):
            p.setJointMotorControl2(bodyIndex=self.armId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=self.rp[i],
                                    force=ARM_MOTOR_FORCE)

        # Initialize gripper with motor control in closed position
        p.setJointMotorControl2(bodyIndex=self.armId,
                                jointIndex=FINGER_JOINT1_INDEX,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=GRIPPER_CLOSED_POSITION,
                                force=GRIPPER_MOTOR_FORCE)
        p.setJointMotorControl2(bodyIndex=self.armId,
                                jointIndex=FINGER_JOINT2_INDEX,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=GRIPPER_CLOSED_POSITION,
                                force=GRIPPER_MOTOR_FORCE)

        for _ in range(STABILIZATION_STEPS):
            p.stepSimulation()

    def open_gripper(self, force=GRIPPER_MOTOR_FORCE):
        """
        Opens the gripper by moving both finger joints to their upper limits.

        Args:
            force: Motor force to apply (default: GRIPPER_MOTOR_FORCE)
        """
        # Update gripper state tracking
        self.current_gripper_pos = GRIPPER_OPEN_POSITION

        if self.logger:
            self.logger.log_robot_gripper("OPEN", GRIPPER_OPEN_POSITION)

        # Allow normal speed for gripper opening (no maxVelocity constraint)
        p.setJointMotorControl2(bodyUniqueId=self.armId, controlMode=p.POSITION_CONTROL,
                                jointIndex=FINGER_JOINT1_INDEX,
                                targetPosition=GRIPPER_OPEN_POSITION,
                                force=force,
                                maxVelocity=0.2)  # Use URDF velocity limit
        p.setJointMotorControl2(bodyUniqueId=self.armId, controlMode=p.POSITION_CONTROL,
                                jointIndex=FINGER_JOINT2_INDEX,
                                targetPosition=GRIPPER_OPEN_POSITION,
                                force=force,
                                maxVelocity=0.2)

        for _ in range(GRIPPER_MOVEMENT_STEPS):
            p.stepSimulation()
            self.simulation_state.step_time()

            ls = p.getLinkState(self.armId, self.endEffectorIndex)
            pos = ls[0]
            if self.simulation_state.hasPrevPose:
                p.addUserDebugLine(self.simulation_state.prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
                p.addUserDebugLine(self.simulation_state.prevPose1, ls[0], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
            self.simulation_state.prevPose = pos
            self.simulation_state.prevPose1 = ls[0]
            self.simulation_state.hasPrevPose = 1

    def close_gripper(self, force=GRIPPER_MOTOR_FORCE):
        """
        Closes the gripper by moving both finger joints to their lower limits.

        Args:
            force: Motor force to apply (default: GRIPPER_MOTOR_FORCE)
        """
        # Update gripper state tracking
        self.current_gripper_pos = GRIPPER_CLOSED_POSITION

        if self.logger:
            self.logger.log_robot_gripper("CLOSE", GRIPPER_CLOSED_POSITION)

        # Allow normal speed for gripper closing (no maxVelocity constraint)
        p.setJointMotorControl2(bodyUniqueId=self.armId, controlMode=p.POSITION_CONTROL,
                                jointIndex=FINGER_JOINT1_INDEX,
                                targetPosition=GRIPPER_CLOSED_POSITION,
                                force=force,
                                maxVelocity=0.2)  # Use URDF velocity limit
        p.setJointMotorControl2(bodyUniqueId=self.armId, controlMode=p.POSITION_CONTROL,
                                jointIndex=FINGER_JOINT2_INDEX,
                                targetPosition=GRIPPER_CLOSED_POSITION,
                                force=force,
                                maxVelocity=0.2)

        for _ in range(GRIPPER_MOVEMENT_STEPS):
            p.stepSimulation()
            self.simulation_state.step_time()

            ls = p.getLinkState(self.armId, self.endEffectorIndex)
            pos = ls[0]
            if self.simulation_state.hasPrevPose:
                p.addUserDebugLine(self.simulation_state.prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
                p.addUserDebugLine(self.simulation_state.prevPose1, ls[0], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
            self.simulation_state.prevPose = pos
            self.simulation_state.prevPose1 = ls[0]
            self.simulation_state.hasPrevPose = 1

    def move_to_target(self, pos, threshold):
        """
        Moves to target position until convergence threshold is met.

        Args:
            pos: Target position [x, y, z]
            threshold: Distance threshold for considering target reached

        Returns:
            gripper_pos: Final gripper position
        """
        if self.logger:
            self.logger.log_robot_operation_start("move_to_target", target_position=pos, threshold=threshold)

        prev_distance = 9999.9999
        s = 0
        max_iterations = MAX_ITERATIONS

        while s < max_iterations:
            s += 1

            if self.useRealTimeSimulation:
                dt = datetime.now()
                self.simulation_state.t = (dt.second / 60.0) * 2.0 * math.pi
            else:
                self.simulation_state.step_time()

            if self.useSimulation and self.useRealTimeSimulation == 0:
                p.stepSimulation()

                if self.useNullSpace == 1:
                    if self.useOrientation == 1:
                        jointPoses = p.calculateInverseKinematics(self.armId, self.endEffectorIndex, pos, self.orn, self.ll, self.ul,
                                                                  self.jr, self.rp)
                    else:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  pos,
                                                                  lowerLimits=self.ll,
                                                                  upperLimits=self.ul,
                                                                  jointRanges=self.jr,
                                                                  restPoses=self.rp)
                else:
                    if self.useOrientation == 1:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  pos,
                                                                  self.orn,
                                                                  jointDamping=self.jd,
                                                                  solver=self.ikSolver,
                                                                  maxNumIterations=IK_MAX_ITERATIONS,
                                                                  residualThreshold=IK_RESIDUAL_THRESHOLD)
                    else:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  pos,
                                                                  solver=self.ikSolver)

                if self.useSimulation:
                    for i in range(NUM_ARM_JOINTS):
                        p.setJointMotorControl2(bodyIndex=self.armId,
                                                jointIndex=i,
                                                controlMode=p.POSITION_CONTROL,
                                                targetPosition=jointPoses[i],
                                                targetVelocity=TARGET_VELOCITY,
                                                force=ARM_MOTOR_FORCE,
                                                positionGain=POSITION_GAIN,
                                                velocityGain=VELOCITY_GAIN)

                    # Maintain gripper position during arm movement with stronger control
                    p.setJointMotorControl2(bodyIndex=self.armId,
                                            jointIndex=FINGER_JOINT1_INDEX,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=self.current_gripper_pos,
                                            force=GRIPPER_MOTOR_FORCE,
                                            positionGain=1.0)
                    p.setJointMotorControl2(bodyIndex=self.armId,
                                            jointIndex=FINGER_JOINT2_INDEX,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=self.current_gripper_pos,
                                            force=GRIPPER_MOTOR_FORCE,
                                            positionGain=1.0)

                else:
                    for i in range(NUM_ARM_JOINTS):
                        p.resetJointState(self.armId, i, jointPoses[i])

            ls = p.getLinkState(self.armId, self.endEffectorIndex)
            if self.simulation_state.hasPrevPose:
                p.addUserDebugLine(self.simulation_state.prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
                p.addUserDebugLine(self.simulation_state.prevPose1, ls[0], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
            self.simulation_state.prevPose = pos
            self.simulation_state.prevPose1 = ls[0]
            self.simulation_state.hasPrevPose = 1

            # Check position convergence
            distance = np.linalg.norm(np.array(pos) - np.array(ls[0]))
            distance_change = prev_distance - distance
            position_converged = distance_change < DISTANCE_CHANGE_THRESHOLD and distance < threshold

            # Check orientation convergence if orientation control is enabled
            orientation_converged = True
            if self.useOrientation == 1:
                current_orn = ls[1]  # ls[1] is the orientation quaternion
                # Calculate quaternion difference using dot product
                # If quaternions are close, their dot product is close to 1 or -1
                dot_product = abs(sum(a * b for a, b in zip(self.orn, current_orn)))
                # Angular difference approximation: theta ≈ 2 * arccos(dot_product)
                orientation_error = 2.0 * math.acos(min(1.0, dot_product))
                orientation_converged = orientation_error < ORIENTATION_THRESHOLD

            if position_converged and orientation_converged:
                if self.logger:
                    self.logger.log_robot_convergence(s, distance)
                    self.logger.log_robot_operation_end("move_to_target", success=True)
                return ls[0]

            prev_distance = distance

        if self.logger:
            self.logger.log_robot_convergence(s, distance)
            self.logger.log_robot_operation_end("move_to_target", success=True)
        return ls[0]

    def move_to_target_smooth(self, pos, threshold):
        """
        Moves to target position in a straight line by calculating the exact next waypoint
        for each step of t, similar to how the original PyBullet example calculates circular motion.
        Each increment of t produces a new waypoint along the straight line.

        Args:
            pos: Target position [x, y, z]
            threshold: Distance threshold for considering target reached

        Returns:
            gripper_pos: Final gripper position
        """
        if self.logger:
            self.logger.log_robot_operation_start("move_to_target_smooth", target_position=pos, threshold=threshold, speed=LINEAR_MOVEMENT_SPEED)

        # Get initial position and starting time
        ls = p.getLinkState(self.armId, self.endEffectorIndex)
        start_pos = np.array(ls[0])
        target_pos = np.array(pos)
        t_start = self.simulation_state.t

        # Calculate total distance
        total_distance = np.linalg.norm(target_pos - start_pos)
        if total_distance < 0.0001:
            return ls[0]

        prev_distance = 9999.9999
        s = 0
        max_iterations = MAX_ITERATIONS

        while s < max_iterations:
            s += 1

            if self.useRealTimeSimulation:
                dt = datetime.now()
                self.simulation_state.t = (dt.second / 60.0) * 2.0 * math.pi
            else:
                self.simulation_state.step_time()

            # Calculate the exact next position based on t with constant velocity
            # In ex.py: pos = [0.4, 0.4, t/10] creates vertical line at 0.1 m/s
            # For general straight line: move at constant LINEAR_MOVEMENT_SPEED along direction

            elapsed_t = self.simulation_state.t - t_start

            # Calculate distance traveled at constant speed
            # Don't overshoot the target
            direction = (target_pos - start_pos) / total_distance
            distance_traveled = min(elapsed_t * LINEAR_MOVEMENT_SPEED, total_distance)

            # Calculate position: start + direction * distance_traveled
            next_pos = start_pos + direction * distance_traveled
            next_pos_list = next_pos.tolist()

            if self.useSimulation and self.useRealTimeSimulation == 0:
                p.stepSimulation()

                if self.useNullSpace == 1:
                    if self.useOrientation == 1:
                        jointPoses = p.calculateInverseKinematics(self.armId, self.endEffectorIndex, next_pos_list, self.orn, self.ll, self.ul,
                                                                  self.jr, self.rp)
                    else:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  next_pos_list,
                                                                  lowerLimits=self.ll,
                                                                  upperLimits=self.ul,
                                                                  jointRanges=self.jr,
                                                                  restPoses=self.rp)
                else:
                    if self.useOrientation == 1:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  next_pos_list,
                                                                  self.orn,
                                                                  jointDamping=self.jd,
                                                                  solver=self.ikSolver,
                                                                  maxNumIterations=IK_MAX_ITERATIONS,
                                                                  residualThreshold=IK_RESIDUAL_THRESHOLD)
                    else:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  next_pos_list,
                                                                  solver=self.ikSolver)

                if self.useSimulation:
                    for i in range(NUM_ARM_JOINTS):
                        p.setJointMotorControl2(bodyIndex=self.armId,
                                                jointIndex=i,
                                                controlMode=p.POSITION_CONTROL,
                                                targetPosition=jointPoses[i],
                                                targetVelocity=TARGET_VELOCITY,
                                                force=ARM_MOTOR_FORCE,
                                                positionGain=POSITION_GAIN_SMOOTH,
                                                velocityGain=VELOCITY_GAIN)

                    # Maintain gripper position during arm movement with stronger control
                    p.setJointMotorControl2(bodyIndex=self.armId,
                                            jointIndex=FINGER_JOINT1_INDEX,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=self.current_gripper_pos,
                                            force=GRIPPER_MOTOR_FORCE,
                                            positionGain=1.0)
                    p.setJointMotorControl2(bodyIndex=self.armId,
                                            jointIndex=FINGER_JOINT2_INDEX,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=self.current_gripper_pos,
                                            force=GRIPPER_MOTOR_FORCE,
                                            positionGain=1.0)

                else:
                    for i in range(NUM_ARM_JOINTS):
                        p.resetJointState(self.armId, i, jointPoses[i])

            ls = p.getLinkState(self.armId, self.endEffectorIndex)
            if self.simulation_state.hasPrevPose:
                p.addUserDebugLine(self.simulation_state.prevPose, next_pos_list, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
                p.addUserDebugLine(self.simulation_state.prevPose1, ls[0], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
            self.simulation_state.prevPose = next_pos_list
            self.simulation_state.prevPose1 = ls[0]
            self.simulation_state.hasPrevPose = 1

            # Check position convergence
            distance = np.linalg.norm(target_pos - np.array(ls[0]))
            distance_change = prev_distance - distance
            position_converged = distance_change < DISTANCE_CHANGE_THRESHOLD and distance < threshold

            # Check orientation convergence if orientation control is enabled
            orientation_converged = True
            if self.useOrientation == 1:
                current_orn = ls[1]  # ls[1] is the orientation quaternion
                # Calculate quaternion difference using dot product
                # If quaternions are close, their dot product is close to 1 or -1
                dot_product = abs(sum(a * b for a, b in zip(self.orn, current_orn)))
                # Angular difference approximation: theta ≈ 2 * arccos(dot_product)
                orientation_error = 2.0 * math.acos(min(1.0, dot_product))
                orientation_converged = orientation_error < ORIENTATION_THRESHOLD

            if position_converged and orientation_converged:
                if self.logger:
                    self.logger.log_robot_convergence(s, distance)
                    self.logger.log_robot_operation_end("move_to_target_smooth", success=True)
                return ls[0]

            prev_distance = distance

        if self.logger:
            self.logger.log_robot_convergence(s, distance)
            self.logger.log_robot_operation_end("move_to_target_smooth", success=True)
        return ls[0]

    def pick_up(self, target_object):
        """
        Picks up a target object by positioning the gripper, opening it, moving to the object,
        closing the gripper, and lifting the object.

        Args:
            target_object: Name of the object to pick up (e.g., 'blue_cube')

        Returns:
            gripper_pos: Final gripper position
        """
        over_target_pos = self.object_manager.get_object_center_position(target_object)

        if self.logger:
            self.logger.log_robot_operation_start("pick_up", object=target_object, object_position=over_target_pos)
            self.logger.log_robot_pick(target_object, over_target_pos)
        over_target_pos[2] = OVER_TARGET_Z
        close_target_pos = self.object_manager.get_object_center_position(target_object)
        close_target_pos[2] += PICK_CLOSE_OFFSET
        target_pos = self.object_manager.get_object_center_position(target_object)
        target_pos[2] += PICK_TARGET_OFFSET


        self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)
        self.move_to_target(close_target_pos, THRESHOLD_CLOSE_TARGET)
        self.open_gripper()
        self.move_to_target_smooth(target_pos, THRESHOLD_PRECISE_STRICT)
        self.close_gripper()
        self.move_to_target_smooth(close_target_pos, THRESHOLD_PRECISE)

        # Capture panorama after pick up operation
        if self.camera_manager is not None:
            self.camera_manager.capture_and_save_panorama(f"pickup_{target_object}")

        gripper_pos = self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)

        if self.logger:
            self.logger.log_robot_operation_end("pick_up", success=True)

        return gripper_pos

    def place(self, place_position):
        """
        Places the currently grasped object at a target position by moving to the location,
        opening the gripper to release the object, and retracting.

        Args:
            place_position: Target position [x, y, z] where the object should be placed

        Returns:
            gripper_pos: Final gripper position
        """
        if self.logger:
            self.logger.log_robot_operation_start("place", position=place_position)
            self.logger.log_robot_place(place_position)

        over_target_pos = place_position.copy()
        over_target_pos[2] = OVER_TARGET_Z
        close_target_pos = place_position.copy()
        close_target_pos[2] += PLACE_CLOSE_OFFSET
        target_pos = place_position.copy()
        target_pos[2] += PLACE_TARGET_OFFSET

        self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)
        self.move_to_target(close_target_pos, THRESHOLD_CLOSE_TARGET)
        self.move_to_target_smooth(target_pos, THRESHOLD_PRECISE_STRICT)
        self.open_gripper()
        self.move_to_target_smooth(close_target_pos, THRESHOLD_PRECISE)
        self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)
        self.close_gripper()

        # Capture panorama after place operation
        if self.camera_manager is not None:
            place_str = f"place_{place_position[0]:.2f}_{place_position[1]:.2f}_{place_position[2]:.2f}"
            self.camera_manager.capture_and_save_panorama(place_str)

        gripper_pos = self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)

        if self.logger:
            self.logger.log_robot_operation_end("place", success=True)

        return gripper_pos

    def place_on(self, target_object):
        """
        Places the currently grasped object on another object.

        Args:
            target_object: The object to place the held object on.

        Returns:
            gripper_pos: Final gripper position
        """
        over_target_pos = self.object_manager.get_object_center_position(target_object)

        if self.logger:
            self.logger.log_robot_operation_start("place_on", object=target_object, object_position=over_target_pos)
            self.logger.log_robot_place(over_target_pos, on_object=target_object)

        over_target_pos[2] = OVER_TARGET_Z
        close_target_pos = self.object_manager.get_object_center_position(target_object)
        close_target_pos[2] += PLACE_ON_CLOSE_OFFSET
        target_pos = self.object_manager.get_object_center_position(target_object)
        target_pos[2] += PLACE_ON_TARGET_OFFSET

        self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)
        self.move_to_target(close_target_pos, THRESHOLD_CLOSE_TARGET)
        self.move_to_target_smooth(target_pos, THRESHOLD_PRECISE_STRICT)
        self.open_gripper()
        self.move_to_target_smooth(close_target_pos, THRESHOLD_PRECISE)
        self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)
        self.close_gripper()

        # Capture panorama after place on operation
        if self.camera_manager is not None:
            self.camera_manager.capture_and_save_panorama(f"place_on_{target_object}")

        gripper_pos = self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)

        if self.logger:
            self.logger.log_robot_operation_end("place_on", success=True)

        return gripper_pos
