import numpy as np
import pybullet as p
import math
from datetime import datetime
from config import *


class RobotController:
    """
    Controls robot movements, gripper operations, and high-level pick-and-place tasks.
    Encapsulates all robot control logic including IK, motion planning, and gripper control.
    """

    def __init__(self, armId, endEffectorIndex, simulation_state, object_manager, camera_manager=None):
        """
        Initialize robot controller.

        Args:
            armId: PyBullet robot body ID
            endEffectorIndex: Index of the end effector link
            simulation_state: SimulationState instance for state tracking
            object_manager: ObjectManager instance for object queries
            camera_manager: CameraManager instance for panorama capture (optional)
        """
        self.armId = armId
        self.endEffectorIndex = endEffectorIndex
        self.simulation_state = simulation_state
        self.object_manager = object_manager
        self.camera_manager = camera_manager

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

    def stabilize(self):
        """Reset robot to rest pose and stabilize with motors."""
        for i in range(NUM_ARM_JOINTS):
            p.resetJointState(self.armId, i, self.rp[i])

        for i in range(NUM_ARM_JOINTS):
            p.setJointMotorControl2(bodyIndex=self.armId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=self.rp[i],
                                    force=ARM_MOTOR_FORCE)

        for _ in range(STABILIZATION_STEPS):
            p.stepSimulation()

    def open_gripper(self, force=GRIPPER_MOTOR_FORCE):
        """Opens the gripper by moving both finger joints to their upper limits."""
        print("=== OPEN GRIPPER START ===")
        print(f"t={self.simulation_state.t}")

        arm_positions = [p.getJointState(self.armId, i)[0] for i in range(NUM_ARM_JOINTS)]

        p.setJointMotorControl2(bodyUniqueId=self.armId, controlMode=p.POSITION_CONTROL,
                                jointIndex=FINGER_JOINT1_INDEX,
                                targetPosition=GRIPPER_OPEN_POSITION,
                                force=force)
        p.setJointMotorControl2(bodyUniqueId=self.armId, controlMode=p.POSITION_CONTROL,
                                jointIndex=FINGER_JOINT2_INDEX,
                                targetPosition=GRIPPER_OPEN_POSITION,
                                force=force)

        for _ in range(GRIPPER_MOVEMENT_STEPS):
            for i in range(NUM_ARM_JOINTS):
                p.setJointMotorControl2(bodyIndex=self.armId,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=arm_positions[i],
                                        force=ARM_MOTOR_FORCE)
            p.stepSimulation()
            self.simulation_state.step_time()

            ls = p.getLinkState(self.armId, self.endEffectorIndex)
            pos = ls[0]
            if self.simulation_state.hasPrevPose:
                p.addUserDebugLine(self.simulation_state.prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
                p.addUserDebugLine(self.simulation_state.prevPose1, ls[4], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
            self.simulation_state.prevPose = pos
            self.simulation_state.prevPose1 = ls[4]
            self.simulation_state.hasPrevPose = 1

        print("=== OPEN GRIPPER END ===")
        print(f"t={self.simulation_state.t}")

    def close_gripper(self, force=GRIPPER_MOTOR_FORCE):
        """Closes the gripper by moving both finger joints to their lower limits."""
        print("=== CLOSE GRIPPER START ===")
        print(f"t={self.simulation_state.t}")

        arm_positions = [p.getJointState(self.armId, i)[0] for i in range(NUM_ARM_JOINTS)]

        p.setJointMotorControl2(bodyUniqueId=self.armId, controlMode=p.POSITION_CONTROL,
                                jointIndex=FINGER_JOINT1_INDEX,
                                targetPosition=GRIPPER_CLOSED_POSITION,
                                force=force)
        p.setJointMotorControl2(bodyUniqueId=self.armId, controlMode=p.POSITION_CONTROL,
                                jointIndex=FINGER_JOINT2_INDEX,
                                targetPosition=GRIPPER_CLOSED_POSITION,
                                force=force)

        for _ in range(GRIPPER_MOVEMENT_STEPS):
            for i in range(NUM_ARM_JOINTS):
                p.setJointMotorControl2(bodyIndex=self.armId,
                                        jointIndex=i,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=arm_positions[i],
                                        force=ARM_MOTOR_FORCE)
            p.stepSimulation()
            self.simulation_state.step_time()

            ls = p.getLinkState(self.armId, self.endEffectorIndex)
            pos = ls[0]
            if self.simulation_state.hasPrevPose:
                p.addUserDebugLine(self.simulation_state.prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
                p.addUserDebugLine(self.simulation_state.prevPose1, ls[4], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
            self.simulation_state.prevPose = pos
            self.simulation_state.prevPose1 = ls[4]
            self.simulation_state.hasPrevPose = 1

        print("=== CLOSE GRIPPER END ===")
        print(f"t={self.simulation_state.t}")

    def move_to_target(self, pos, threshold):
        """
        Moves to target position until convergence threshold is met.

        Args:
            pos: Target position [x, y, z]
            threshold: Distance threshold for considering target reached

        Returns:
            gripper_pos: Final gripper position
        """
        print(f"=== MOVE TO TARGET START === Target: {pos}, Threshold: {threshold}")
        print(f"t={self.simulation_state.t}")

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
                orn = p.getQuaternionFromEuler([0.0, -math.pi, 0.0])

                if self.useNullSpace == 1:
                    if self.useOrientation == 1:
                        jointPoses = p.calculateInverseKinematics(self.armId, self.endEffectorIndex, pos, orn, self.ll, self.ul,
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
                                                                  orn,
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
                else:
                    for i in range(NUM_ARM_JOINTS):
                        p.resetJointState(self.armId, i, jointPoses[i])

            ls = p.getLinkState(self.armId, self.endEffectorIndex)
            if self.simulation_state.hasPrevPose:
                p.addUserDebugLine(self.simulation_state.prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
                p.addUserDebugLine(self.simulation_state.prevPose1, ls[4], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
            self.simulation_state.prevPose = pos
            self.simulation_state.prevPose1 = ls[4]
            self.simulation_state.hasPrevPose = 1
            distance = np.linalg.norm(np.array(pos) - np.array(ls[0]))
            distance_change = prev_distance - distance

            if distance_change < DISTANCE_CHANGE_THRESHOLD and distance < threshold:
                print(f"=== MOVE TO TARGET END === Result: CONVERGED, Final distance: {distance:.6f}, Iterations: {s}")
                print(f"t={self.simulation_state.t}")
                return ls[0]

            prev_distance = distance

        print(f"=== MOVE TO TARGET END === Result: MAX_ITERATIONS_REACHED, Final distance: {distance:.6f}, Iterations: {s}")
        print(f"t={self.simulation_state.t}")
        return ls[0]

    def move_to_target_smooth(self, pos, threshold):
        """
        Moves to target position using linear interpolation in each iteration.
        In every step, the arm moves to the next point along a straight line from start to target.
        Uses time-based interpolation for consistent movement speed.

        Args:
            pos: Target position [x, y, z]
            threshold: Distance threshold for considering target reached

        Returns:
            gripper_pos: Final gripper position
        """
        print(f"=== MOVE TO TARGET SMOOTH START === Target: {pos}, Threshold: {threshold}")
        print(f"t={self.simulation_state.t}")

        # Get initial position at the start
        ls = p.getLinkState(self.armId, self.endEffectorIndex)
        start_pos = np.array(ls[0])
        target_pos = np.array(pos)

        # Calculate total distance for speed adjustment
        total_distance = np.linalg.norm(target_pos - start_pos)

        # Scale movement speed based on distance to maintain straightness
        # Longer distances need more time for the arm to follow the path accurately
        # This prevents the arm from lagging too far behind the moving target
        movement_duration = total_distance * SMOOTH_MOVEMENT_SPEED_MULTIPLIER

        prev_distance = 9999.9999
        s = 0
        max_iterations = MAX_ITERATIONS

        # Time parameter for smooth interpolation
        movement_time = 0.0

        while s < max_iterations:
            s += 1

            if self.useRealTimeSimulation:
                dt = datetime.now()
                self.simulation_state.t = (dt.second / 60.0) * 2.0 * math.pi
            else:
                self.simulation_state.step_time()

            # Increment time by fixed step (like PyBullet example)
            movement_time = movement_time + TIME_STEP

            # Calculate linear interpolation factor (0 to 1), scaled by distance
            # Slower progression for longer distances to keep the arm closer to the target
            alpha = min(1.0, movement_time / movement_duration)

            # Calculate intermediate target position using linear interpolation
            intermediate_pos = start_pos + alpha * (target_pos - start_pos)
            intermediate_pos_list = intermediate_pos.tolist()

            if self.useSimulation and self.useRealTimeSimulation == 0:
                p.stepSimulation()
                orn = p.getQuaternionFromEuler([0.0, -math.pi, 0.0])

                if self.useNullSpace == 1:
                    if self.useOrientation == 1:
                        jointPoses = p.calculateInverseKinematics(self.armId, self.endEffectorIndex, intermediate_pos_list, orn, self.ll, self.ul,
                                                                  self.jr, self.rp)
                    else:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  intermediate_pos_list,
                                                                  lowerLimits=self.ll,
                                                                  upperLimits=self.ul,
                                                                  jointRanges=self.jr,
                                                                  restPoses=self.rp)
                else:
                    if self.useOrientation == 1:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  intermediate_pos_list,
                                                                  orn,
                                                                  jointDamping=self.jd,
                                                                  solver=self.ikSolver,
                                                                  maxNumIterations=IK_MAX_ITERATIONS,
                                                                  residualThreshold=IK_RESIDUAL_THRESHOLD)
                    else:
                        jointPoses = p.calculateInverseKinematics(self.armId,
                                                                  self.endEffectorIndex,
                                                                  intermediate_pos_list,
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
                else:
                    for i in range(NUM_ARM_JOINTS):
                        p.resetJointState(self.armId, i, jointPoses[i])

            ls = p.getLinkState(self.armId, self.endEffectorIndex)
            if self.simulation_state.hasPrevPose:
                p.addUserDebugLine(self.simulation_state.prevPose, intermediate_pos_list, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
                p.addUserDebugLine(self.simulation_state.prevPose1, ls[4], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, self.simulation_state.trailDuration)
            self.simulation_state.prevPose = intermediate_pos_list
            self.simulation_state.prevPose1 = ls[4]
            self.simulation_state.hasPrevPose = 1
            distance = np.linalg.norm(target_pos - np.array(ls[0]))
            distance_change = prev_distance - distance

            if distance_change < DISTANCE_CHANGE_THRESHOLD and distance < threshold:
                print(f"=== MOVE TO TARGET SMOOTH END === Result: CONVERGED, Final distance: {distance:.6f}, Iterations: {s}")
                print(f"t={self.simulation_state.t}")
                return ls[0]

            prev_distance = distance

        print(f"=== MOVE TO TARGET SMOOTH END === Result: MAX_ITERATIONS_REACHED, Final distance: {distance:.6f}, Iterations: {s}")
        print(f"t={self.simulation_state.t}")
        return ls[0]

    def move_to_target_linear(self, pos, threshold):
        """
        Moves to target position using linear interpolation through intermediate waypoints.
        This results in a straighter trajectory than direct move_to_target.
        Waypoints are automatically calculated to have one waypoint per centimeter of distance.

        Args:
            pos: Target position [x, y, z]
            threshold: Distance threshold for considering target reached

        Returns:
            gripper_pos: Final gripper position
        """
        ls = p.getLinkState(self.armId, self.endEffectorIndex)
        current_pos = ls[0]

        # Calculate distance in meters
        distance = np.linalg.norm(np.array(pos) - np.array(current_pos))

        # Convert to centimeters and round up to get number of waypoints
        distance_cm = distance * 100
        num_waypoints = max(1, int(np.ceil(distance_cm)))

        print(f"=== MOVE TO TARGET LINEAR START === Target: {pos}, Distance: {distance:.4f}m ({distance_cm:.2f}cm), Waypoints: {num_waypoints}")
        print(f"Current end effector position: {current_pos}, t={self.simulation_state.t}")

        for i in range(1, num_waypoints + 1):
            alpha = i / num_waypoints

            waypoint = [
                current_pos[0] + alpha * (pos[0] - current_pos[0]),
                current_pos[1] + alpha * (pos[1] - current_pos[1]),
                current_pos[2] + alpha * (pos[2] - current_pos[2])
            ]

            distance_to_target = np.linalg.norm(np.array(pos) - np.array(waypoint))
            print(f"waypoint {i}/{num_waypoints} Target: {waypoint}, Distance to final: {distance_to_target:.4f}")
            print(f"t={self.simulation_state.t}")

            gripper_pos = self.move_to_target(waypoint, threshold)

        print("=== MOVE TO TARGET LINEAR END ===")
        print(f"t={self.simulation_state.t}")
        return gripper_pos

    def pick_up(self, target_object):
        """
        Picks up a target object by positioning the gripper, opening it, moving to the object,
        closing the gripper, and lifting the object.

        Args:
            target_object: Name of the object to pick up (e.g., 'blue_cube')

        Returns:
            gripper_pos: Final gripper position
        """
        print(f"=== PICK UP START === Object: {target_object}")
        print(f"t={self.simulation_state.t}")

        over_target_pos = self.object_manager.get_object_center_position(target_object)
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

        print("=== PICK UP END ===")
        print(f"t={self.simulation_state.t}")

        # Capture panorama after pick up operation
        if self.camera_manager is not None:
            self.camera_manager.capture_and_save_panorama(f"pickup_{target_object}")

        gripper_pos = self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)
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
        print(f"=== PLACE START === Position: {place_position}")
        print(f"t={self.simulation_state.t}")

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

        print("=== PLACE END ===")
        print(f"t={self.simulation_state.t}")

        # Capture panorama after place operation
        if self.camera_manager is not None:
            place_str = f"place_{place_position[0]:.2f}_{place_position[1]:.2f}_{place_position[2]:.2f}"
            self.camera_manager.capture_and_save_panorama(place_str)

        gripper_pos = self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)
        return gripper_pos

    def place_on(self, target_object):
        """
        Places the currently grasped object on another object.

        Args:
            target_object: The object to place the held object on.

        Returns:
            gripper_pos: Final gripper position
        """
        print(f"=== PLACE_ON START === Target object: {target_object}")
        print(f"t={self.simulation_state.t}")

        over_target_pos = self.object_manager.get_object_center_position(target_object)
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

        print("=== PLACE ON END ===")
        print(f"t={self.simulation_state.t}")

        # Capture panorama after place on operation
        if self.camera_manager is not None:
            self.camera_manager.capture_and_save_panorama(f"place_on_{target_object}")

        gripper_pos = self.move_to_target(over_target_pos, THRESHOLD_OVER_TARGET)
        return gripper_pos
