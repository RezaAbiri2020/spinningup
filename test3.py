# a test for jaco in pybullet

import os
import pybullet as p
import pybullet_data
import math 

# connecting to pybullet
p.connect(p.GUI)
urdfRootPath=pybullet_data.getDataPath()

# adding the robot
jacoUid = p.loadURDF(os.path.join(urdfRootPath,"jaco/j2s7s300_gym.urdf"), useFixedBase=True)

JointNum=p.getNumJoints(jacoUid)
print('Number of joints are (starts with 0):')
print(JointNum)

JointInfo=p.getJointInfo(jacoUid,14)
print('Joint information are:')
print(JointInfo)

# adding the table 
#tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])

# adding the tray
#trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.65,0,0])

# adding a random object for grasping
#p.setGravity(0,0,-10)
#objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=[0.7,0,0.1])

# changing the view for camera 
p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])


#for controlling 7 joints
p.setJointMotorControl2(jacoUid, 1, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 2, p.POSITION_CONTROL,math.pi/1)
p.setJointMotorControl2(jacoUid, 3, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 4, p.POSITION_CONTROL,-math.pi/2)
p.setJointMotorControl2(jacoUid, 5, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 6, p.POSITION_CONTROL,math.pi/1)
p.setJointMotorControl2(jacoUid, 7, p.POSITION_CONTROL, 0)

# these joints are not controllable; these are the tips of fingers
p.setJointMotorControl2(jacoUid, 10, p.POSITION_CONTROL,math.pi/1)
p.setJointMotorControl2(jacoUid, 12, p.POSITION_CONTROL,math.pi/1)
p.setJointMotorControl2(jacoUid, 14, p.POSITION_CONTROL,math.pi/1)

'''
# for controlling 3 fingers
p.setJointMotorControl2(jacoUid, 9, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 11, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 13, p.POSITION_CONTROL, 0)
'''


while True:
    p.stepSimulation()
