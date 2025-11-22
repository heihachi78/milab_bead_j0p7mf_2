from re import S
import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data

clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)
  #p.connect(p.SHARED_MEMORY_GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf", [0, 0, -0.3])
pandaId = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(pandaId, [0, 0, 0], [0, 0, 0, 1])
pandaEndEffectorIndex = 11
numJoints = p.getNumJoints(pandaId)
print(f"Number of joints: {numJoints}")

#lower limits for null space (Franka Panda limits)
ll = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
#upper limits for null space
ul = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
#joint ranges for null space
jr = [5.8, 3.5, 5.8, 3.0, 5.8, 3.8, 5.8]
#restposes for null space
rp = [0, 0, 0, -1.5, 0, 1.5, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# Reset arm joints to rest poses
for i in range(7):  # Only first 7 joints are arm joints
  p.resetJointState(pandaId, i, rp[i])

p.setGravity(0, 0, 0)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 1

useOrientation = 0
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 0
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

x=0
while 1:
  x+=1
  #p.getCameraImage(320,
  #                 200,
  #                 flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
  #                 renderer=p.ER_BULLET_HARDWARE_OPENGL)
  if (useRealTimeSimulation):
    dt = datetime.now()
    t = (dt.second / 60.) * 2. * math.pi
  else:
    t = t + 0.01

  if (useSimulation and useRealTimeSimulation == 0):
    p.stepSimulation()

  for i in range(1):
    #pos = [-0.4, 0.2 * math.cos(t), 0. + 0.2 * math.sin(t)]
    if x < 100:
      pos = [0.4, 0.4, 0]
    else:
      pos = [0.4, 0.4, t/10]
    #end effector points down, not up (in case useOrientation==1)
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    if (useNullSpace == 1):
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(pandaId, pandaEndEffectorIndex, pos, orn, ll, ul,
                                                  jr, rp)
      else:
        jointPoses = p.calculateInverseKinematics(pandaId,
                                                  pandaEndEffectorIndex,
                                                  pos,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)
    else:
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(pandaId,
                                                  pandaEndEffectorIndex,
                                                  pos,
                                                  orn,
                                                  jointDamping=jd,
                                                  solver=ikSolver,
                                                  maxNumIterations=100,
                                                  residualThreshold=.01)
      else:
        jointPoses = p.calculateInverseKinematics(pandaId,
                                                  pandaEndEffectorIndex,
                                                  pos,
                                                  solver=ikSolver)

    if (useSimulation):
      for i in range(7):  # Only control first 7 joints (arm joints)
        p.setJointMotorControl2(bodyIndex=pandaId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)
    else:
      #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
      for i in range(7):  # Only reset first 7 joints (arm joints)
        p.resetJointState(pandaId, i, jointPoses[i])

  ls = p.getLinkState(pandaId, pandaEndEffectorIndex)
  if (hasPrevPose):
    p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
    p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
  prevPose = pos
  prevPose1 = ls[4]
  hasPrevPose = 1
p.disconnect()
