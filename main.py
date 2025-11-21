import numpy as np
import pybullet as p
import math
from datetime import datetime
import pybullet_data
from config import *

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  # p.connect(p.SHARED_MEMORY_GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", PLANE_POSITION)
armId = p.loadURDF("franka_panda/panda.urdf", ROBOT_BASE_POSITION, useFixedBase=True)
p.resetBasePositionAndOrientation(armId, ROBOT_BASE_POSITION, ROBOT_BASE_ORIENTATION)
endEffectorIndex = END_EFFECTOR_INDEX  # Franka Panda grasp-target link
numJoints = p.getNumJoints(armId)
print(f"Number of joints: {numJoints}")

# Lower limits for null space (Franka Panda 7 arm joints)
ll = LOWER_LIMITS
# Upper limits for null space
ul = UPPER_LIMITS
# Joint ranges for null space
jr = JOINT_RANGES
# Rest poses for null space (stable neutral position)
rp = REST_POSES
# Joint damping coefficients
jd = JOINT_DAMPING

# Only reset the 7 arm joints, not gripper joints
for i in range(NUM_ARM_JOINTS):
  p.resetJointState(armId, i, rp[i])

p.setGravity(0, 0, GRAVITY)

# Stabilize the robot with motors before starting
for i in range(NUM_ARM_JOINTS):
  p.setJointMotorControl2(bodyIndex=armId,
                          jointIndex=i,
                          controlMode=p.POSITION_CONTROL,
                          targetPosition=rp[i],
                          force=ARM_MOTOR_FORCE)

# Let the robot stabilize
for _ in range(STABILIZATION_STEPS):
  p.stepSimulation()
t = INITIAL_TIME
prevPose = INITIAL_PREV_POSE.copy()
prevPose1 = INITIAL_PREV_POSE.copy()
hasPrevPose = INITIAL_HAS_PREV_POSE
useNullSpace = USE_NULL_SPACE

useOrientation = USE_ORIENTATION
# If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
# This can be used to test the IK result accuracy.
useSimulation = USE_SIMULATION
useRealTimeSimulation = USE_REAL_TIME_SIMULATION
ikSolver = IK_SOLVER
p.setRealTimeSimulation(useRealTimeSimulation)
# trailDuration is duration (in seconds) after debug lines will be removed automatically
# use 0 for no-removal
trailDuration = TRAIL_DURATION

obj0 = p.loadURDF("cube_small.urdf", basePosition=BLUE_CUBE_POS, globalScaling=OBJECT_SCALE)
p.changeVisualShape(obj0, GRIPPER_LINK_INDEX, rgbaColor=BLUE_COLOR)  # Blue

obj1 = p.loadURDF("cube_small.urdf", basePosition=RED_CUBE_POS, globalScaling=OBJECT_SCALE)
p.changeVisualShape(obj1, GRIPPER_LINK_INDEX, rgbaColor=RED_COLOR)  # Red

obj2 = p.loadURDF("cube_small.urdf", basePosition=GREEN_CUBE_POS, globalScaling=OBJECT_SCALE)
p.changeVisualShape(obj2, GRIPPER_LINK_INDEX, rgbaColor=GREEN_COLOR)  # Green

def get_object_dimensions(obj_id):
  """
  Dynamically queries object dimensions from PyBullet based on AABB.

  Args:
    obj_id: PyBullet object ID

  Returns:
    [width, depth, height] list with object dimensions
  """
  # Query AABB (Axis-Aligned Bounding Box) - this is always accurate
  aabb_min, aabb_max = p.getAABB(obj_id, -1)

  # Calculate size: max - min for each axis
  width = aabb_max[0] - aabb_min[0]
  depth = aabb_max[1] - aabb_min[1]
  height = aabb_max[2] - aabb_min[2]

  return_size = [width, depth, height]
  print(f"Object {obj_id} size (AABB): {return_size}")
  return return_size

# Object registry - name -> object_id
# Dimensions are queried dynamically when needed
objects = {
  'blue_cube': obj0,
  'red_cube': obj1,
  'green_cube': obj2
}

def get_object_center_position(object_name):
  """
  Returns the center position of an object.

  Args:
    object_name: The name of the object (e.g., 'red_cube')

  Returns:
    [x, y, z] list with the object's center position,
    or None if the object is not found
  """
  if object_name not in objects:
    print(f"Error: Object '{object_name}' not found in registry")
    return None

  obj_id = objects[object_name]

  # Dynamically query object dimensions
  obj_size = get_object_dimensions(obj_id)

  # Object base position
  base_pos, _ = p.getBasePositionAndOrientation(obj_id)

  # Calculate center: base position + half height (z direction)
  # In cube_small.urdf, the cube's bottom is at the base position,
  # so we add half the height
  center_pos = [
    base_pos[0],
    base_pos[1],
    base_pos[2] + obj_size[2] / 2.0  # z + half height
  ]

  print(f"Object '{object_name}' center position: {center_pos}")

  return center_pos

def open_gripper(t, hasPrevPose, prevPose, prevPose1, force=GRIPPER_MOTOR_FORCE):
  """Opens the gripper by moving both finger joints to their upper limits."""
  print("=== OPEN GRIPPER START ===")
  print(f"t={t}")
  # Get current arm joint positions to keep them stable
  arm_positions = [p.getJointState(armId, i)[0] for i in range(NUM_ARM_JOINTS)]

  p.setJointMotorControl2(bodyUniqueId=armId, controlMode=p.POSITION_CONTROL,
                          jointIndex=FINGER_JOINT1_INDEX,  # finger_joint1
                          targetPosition=GRIPPER_OPEN_POSITION,  # upper limit for finger_joint1
                          force=force)
  p.setJointMotorControl2(bodyUniqueId=armId, controlMode=p.POSITION_CONTROL,
                          jointIndex=FINGER_JOINT2_INDEX,  # finger_joint2
                          targetPosition=GRIPPER_OPEN_POSITION,  # upper limit for finger_joint2
                          force=force)

  # Keep arm stable while gripper moves
  for _ in range(GRIPPER_MOVEMENT_STEPS):
    # Maintain arm position
    for i in range(NUM_ARM_JOINTS):
      p.setJointMotorControl2(bodyIndex=armId,
                              jointIndex=i,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=arm_positions[i],
                              force=ARM_MOTOR_FORCE)
    p.stepSimulation()
    t = t + TIME_STEP

    # Draw debug lines
    ls = p.getLinkState(armId, endEffectorIndex)
    pos = ls[0]  # Current end effector position
    if (hasPrevPose):
      p.addUserDebugLine(prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, trailDuration)
      p.addUserDebugLine(prevPose1, ls[4], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

  print("=== OPEN GRIPPER END ===")
  print(f"t={t}")
  return t, hasPrevPose, prevPose, prevPose1

def close_gripper(t, hasPrevPose, prevPose, prevPose1, force=GRIPPER_MOTOR_FORCE):
  """Closes the gripper by moving both finger joints to their lower limits."""
  print("=== CLOSE GRIPPER START ===")
  print(f"t={t}")
  # Get current arm joint positions to keep them stable
  arm_positions = [p.getJointState(armId, i)[0] for i in range(NUM_ARM_JOINTS)]

  p.setJointMotorControl2(bodyUniqueId=armId, controlMode=p.POSITION_CONTROL,
                          jointIndex=FINGER_JOINT1_INDEX,  # finger_joint1
                          targetPosition=GRIPPER_CLOSED_POSITION,  # lower limit for finger_joint1
                          force=force)
  p.setJointMotorControl2(bodyUniqueId=armId, controlMode=p.POSITION_CONTROL,
                          jointIndex=FINGER_JOINT2_INDEX,  # finger_joint2
                          targetPosition=GRIPPER_CLOSED_POSITION,  # lower limit for finger_joint2
                          force=force)

  # Keep arm stable while gripper moves
  for _ in range(GRIPPER_MOVEMENT_STEPS):
    # Maintain arm position
    for i in range(NUM_ARM_JOINTS):
      p.setJointMotorControl2(bodyIndex=armId,
                              jointIndex=i,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=arm_positions[i],
                              force=ARM_MOTOR_FORCE)
    p.stepSimulation()
    t = t + TIME_STEP

    # Draw debug lines
    ls = p.getLinkState(armId, endEffectorIndex)
    pos = ls[0]  # Current end effector position
    if (hasPrevPose):
      p.addUserDebugLine(prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, trailDuration)
      p.addUserDebugLine(prevPose1, ls[4], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

  print("=== CLOSE GRIPPER END ===")
  print(f"t={t}")
  return t, hasPrevPose, prevPose, prevPose1

def move_to_target_linear(pos, t, hasPrevPose, prevPose, prevPose1, threshold, num_waypoints=DEFAULT_NUM_WAYPOINTS):
  """
  Moves to target position using linear interpolation through intermediate waypoints.
  This results in a straighter trajectory than direct move_to_target.

  Calculates intermediate points and calls move_to_target for each waypoint.

  Args:
    pos: Target position [x, y, z]
    num_waypoints: Number of intermediate points on the trajectory (default: 10)
  """
  print(f"=== MOVE TO TARGET LINEAR START === Target: {pos}, Waypoints: {num_waypoints}")

  # Get current end effector position
  ls = p.getLinkState(armId, endEffectorIndex)
  current_pos = ls[0]
  print(f"Current end effector position: {current_pos}, t={t}")

  # Linear interpolation between current and target position
  for i in range(1, num_waypoints + 1):
    # Interpolation ratio (0.0 -> 1.0)
    alpha = i / num_waypoints

    # Calculate intermediate waypoint
    waypoint = [
      current_pos[0] + alpha * (pos[0] - current_pos[0]),
      current_pos[1] + alpha * (pos[1] - current_pos[1]),
      current_pos[2] + alpha * (pos[2] - current_pos[2])
    ]

    # Distance to final target from current waypoint
    distance_to_target = np.linalg.norm(np.array(pos) - np.array(waypoint))
    print(f"waypoint {i}/{num_waypoints} Target: {waypoint}, Distance to final: {distance_to_target:.4f}")
    print(f"t={t}")

    # Call move_to_target for this waypoint
    t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(waypoint, t, hasPrevPose, prevPose, prevPose1, threshold)

  print("=== MOVE TO TARGET LINEAR END ===")
  print(f"t={t}")
  return t, hasPrevPose, prevPose, prevPose1, gripper_pos

def move_to_target(pos, t, hasPrevPose, prevPose, prevPose1, threshold):
  prev_distance = 9999.9999
  s = 0
  max_iterations = MAX_ITERATIONS  # Safety limit

  while s < max_iterations:
    s += 1
    # p.getCameraImage(320,
    #                  200,
    #                  flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
    #                  renderer=p.ER_BULLET_HARDWARE_OPENGL)
    if (useRealTimeSimulation):
      dt = datetime.now()
      t = (dt.second / 60.0) * 2.0 * math.pi
    else:
      t = t + TIME_STEP

    if (useSimulation and useRealTimeSimulation == 0):
      p.stepSimulation()
      # End effector points down, not up (in case useOrientation==1)
      orn = p.getQuaternionFromEuler([0.0, -math.pi, 0.0])

      if (useNullSpace == 1):
        if (useOrientation == 1):
          jointPoses = p.calculateInverseKinematics(armId, endEffectorIndex, pos, orn, ll, ul,
                                                    jr, rp)
        else:
          jointPoses = p.calculateInverseKinematics(armId,
                                                    endEffectorIndex,
                                                    pos,
                                                    lowerLimits=ll,
                                                    upperLimits=ul,
                                                    jointRanges=jr,
                                                    restPoses=rp)
      else:
        if (useOrientation == 1):
          jointPoses = p.calculateInverseKinematics(armId,
                                                    endEffectorIndex,
                                                    pos,
                                                    orn,
                                                    jointDamping=jd,
                                                    solver=ikSolver,
                                                    maxNumIterations=IK_MAX_ITERATIONS,
                                                    residualThreshold=IK_RESIDUAL_THRESHOLD)
        else:
          jointPoses = p.calculateInverseKinematics(armId,
                                                    endEffectorIndex,
                                                    pos,
                                                    solver=ikSolver)

      if (useSimulation):
        # Only control the 7 arm joints, not gripper
        for i in range(NUM_ARM_JOINTS):
          p.setJointMotorControl2(bodyIndex=armId,
                                  jointIndex=i,
                                  controlMode=p.POSITION_CONTROL,
                                  targetPosition=jointPoses[i],
                                  targetVelocity=TARGET_VELOCITY,
                                  force=ARM_MOTOR_FORCE,
                                  positionGain=POSITION_GAIN,
                                  velocityGain=VELOCITY_GAIN)
      else:
        # Reset the joint state (ignoring all dynamics, not recommended to use during simulation)
        for i in range(NUM_ARM_JOINTS):
          p.resetJointState(armId, i, jointPoses[i])

    ls = p.getLinkState(armId, endEffectorIndex)
    if (hasPrevPose):
      p.addUserDebugLine(prevPose, pos, DEBUG_LINE_COLOR_1, DEBUG_LINE_WIDTH, trailDuration)
      p.addUserDebugLine(prevPose1, ls[4], DEBUG_LINE_COLOR_2, DEBUG_LINE_WIDTH, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1
    distance = np.linalg.norm(np.array(pos) - np.array(ls[0]))
    distance_change = prev_distance - distance
    #print(f"Iteration {s}: Position: {pos}, Gripper position: {ls[0]} Distance: {distance:.4f}, Distance change: {distance_change:.6f}, t={t}")
    if distance_change < DISTANCE_CHANGE_THRESHOLD and distance < threshold:
      return t, hasPrevPose, prevPose, prevPose1, ls[0]

    prev_distance = distance

  # If max iterations reached, return current state anyway
  return t, hasPrevPose, prevPose, prevPose1, ls[0]

for i in range(STABILIZATION_LOOP_STEPS):
  p.stepSimulation()


def pick_up(target_object, t, hasPrevPose, prevPose, prevPose1):
  """
  Picks up a target object by positioning the gripper, opening it, moving to the object,
  closing the gripper, and lifting the object.

  Args:
    target_object: Name of the object to pick up (e.g., 'blue_cube')
    t: Current simulation time
    hasPrevPose: Flag indicating if previous pose is available for debug lines
    prevPose: Previous end effector position for debug lines
    prevPose1: Previous link state for debug lines

  Returns:
    Tuple of (t, hasPrevPose, prevPose, prevPose1, gripper_pos) with updated values
  """
  print(f"=== PICK UP START === Object: {target_object}")
  print(f"t={t}")

  target_pos = get_object_center_position(target_object)
  target_pos[2] += PICK_Z_OFFSET_DOWN
  over_target_pos = get_object_center_position(target_object)
  over_target_pos[2] = OVER_TARGET_Z
  gripper_pos = [0, 0, 0]
  close_target_pos = get_object_center_position(target_object)
  close_target_pos[2] += PICK_Z_OFFSET_UP
  gripper_pos = [0, 0, 0]
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(over_target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_OVER_TARGET)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(close_target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_CLOSE_TARGET)
  t, hasPrevPose, prevPose, prevPose1 = open_gripper(t, hasPrevPose, prevPose, prevPose1)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target_linear(target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_PRECISE)
  t, hasPrevPose, prevPose, prevPose1 = close_gripper(t, hasPrevPose, prevPose, prevPose1)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target_linear(close_target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_PRECISE)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(over_target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_OVER_TARGET)

  print("=== PICK UP END ===")
  print(f"t={t}")
  return t, hasPrevPose, prevPose, prevPose1, gripper_pos

def place(place_position, t, hasPrevPose, prevPose, prevPose1):
  """
  Places the currently grasped object at a target position by moving to the location,
  opening the gripper to release the object, and retracting.

  Args:
    place_position: Target position [x, y, z] where the object should be placed
    t: Current simulation time
    hasPrevPose: Flag indicating if previous pose is available for debug lines
    prevPose: Previous end effector position for debug lines
    prevPose1: Previous link state for debug lines

  Returns:
    Tuple of (t, hasPrevPose, prevPose, prevPose1, gripper_pos) with updated values
  """
  print(f"=== PLACE START === Position: {place_position}")
  print(f"t={t}")

  target_pos = place_position.copy()
  target_pos[2] += PLACE_Z_OFFSET_UP
  over_target_pos = place_position.copy()
  over_target_pos[2] = OVER_TARGET_Z
  gripper_pos = [0, 0, 0]
  close_target_pos = place_position.copy()
  close_target_pos[2] += PLACE_Z_OFFSET_CLOSE
  gripper_pos = [0, 0, 0]
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(over_target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_OVER_TARGET)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(close_target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_CLOSE_TARGET)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target_linear(target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_PRECISE)
  t, hasPrevPose, prevPose, prevPose1 = open_gripper(t, hasPrevPose, prevPose, prevPose1)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target_linear(close_target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_PRECISE)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(over_target_pos, t, hasPrevPose, prevPose, prevPose1, THRESHOLD_OVER_TARGET)
  t, hasPrevPose, prevPose, prevPose1 = close_gripper(t, hasPrevPose, prevPose, prevPose1)

  print("=== PLACE END ===")
  print(f"t={t}")
  return t, hasPrevPose, prevPose, prevPose1, gripper_pos


for i in range(PICK_PLACE_ITERATIONS):

  print(f"==========================================")
  print(f"simulation started t={t}")
  print(f"==========================================")

  t, hasPrevPose, prevPose, prevPose1, gripper_pos = pick_up('blue_cube', t, hasPrevPose, prevPose, prevPose1)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = place(get_object_center_position('red_cube'), t, hasPrevPose, prevPose, prevPose1)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = pick_up('green_cube', t, hasPrevPose, prevPose, prevPose1)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = place(get_object_center_position('blue_cube'), t, hasPrevPose, prevPose, prevPose1)

  print(f"==========================================")
  print(f"simulation ended t={t}")
  print(f"==========================================")

'''
  target_pos = get_object_center_position('blue_cube')
  target_pos[2] -= 0.025
  over_target_pos = get_object_center_position('blue_cube')
  over_target_pos[2] = 0.5
  gripper_pos = [0, 0, 0]
  close_target_pos = get_object_center_position('blue_cube')
  close_target_pos[2] += 0.05
  gripper_pos = [0, 0, 0]

  # Move with linear interpolation for straighter trajectory
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(over_target_pos, t, hasPrevPose, prevPose, prevPose1, 0.05)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(close_target_pos, t, hasPrevPose, prevPose, prevPose1, 0.025)
  t, hasPrevPose, prevPose, prevPose1 = open_gripper(t, hasPrevPose, prevPose, prevPose1)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target_linear(target_pos, t, hasPrevPose, prevPose, prevPose1, 0.01)
  t, hasPrevPose, prevPose, prevPose1 = close_gripper(t, hasPrevPose, prevPose, prevPose1)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target_linear(close_target_pos, t, hasPrevPose, prevPose, prevPose1, 0.01)
  t, hasPrevPose, prevPose, prevPose1, gripper_pos = move_to_target(over_target_pos, t, hasPrevPose, prevPose, prevPose1, 0.05)
'''

p.disconnect()
