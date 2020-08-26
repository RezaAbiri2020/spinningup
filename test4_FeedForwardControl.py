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
print('Number of joints are:')
print(JointNum)

JointInfo=p.getJointInfo(jacoUid,0)
print('Joint information are:')
print(JointInfo)

# adding the table 
#tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])

# adding the tray
#trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.65,0,0])

# adding a random object for grasping
p.setGravity(0,0,-10)
objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=[0.7,0,0.1])

# changing the view for camera 
p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])

#p.setJointMotorControl2(jacoUid, 2, p.POSITION_CONTROL,math.pi/1)

'''
#for controlling 7 joints
p.setJointMotorControl2(jacoUid, 1, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 2, p.POSITION_CONTROL,math.pi/1)
p.setJointMotorControl2(jacoUid, 3, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 4, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 5, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 6, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 7, p.POSITION_CONTROL, 0)

# for controlling 3 fingers
p.setJointMotorControl2(jacoUid, 9, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 11, p.POSITION_CONTROL,0)
p.setJointMotorControl2(jacoUid, 13, p.POSITION_CONTROL, 0)
'''

state_durations = [1, 1, 1, 1]
control_dt = 1./240.
p.setTimestep = control_dt
state_t = 0.
current_state = 0

#while True:
#    p.stepSimulation()

# controlling the robot using only state and not based on sample timing
while True:
    state_t += control_dt
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING) 
    if current_state == 0:
        p.setJointMotorControl2(jacoUid, 1, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(jacoUid, 2, 
                        p.POSITION_CONTROL,math.pi/2.)
        p.setJointMotorControl2(jacoUid, 3, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(jacoUid, 4, 
                        p.POSITION_CONTROL,4*math.pi/4.)
        p.setJointMotorControl2(jacoUid, 5, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(jacoUid, 6, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(jacoUid, 7, 
                        p.POSITION_CONTROL,0)
        p.setJointMotorControl2(jacoUid, 9, 
                        p.POSITION_CONTROL, 0.08)
        p.setJointMotorControl2(jacoUid, 11, 
                        p.POSITION_CONTROL, 0.08)
        p.setJointMotorControl2(jacoUid, 13, 
                        p.POSITION_CONTROL, 0.08)
                        
    if current_state == 1:
        p.setJointMotorControl2(jacoUid, 2, 
                        p.POSITION_CONTROL,math.pi/2.-.5)
        p.setJointMotorControl2(jacoUid, 4, 
                        p.POSITION_CONTROL,4*math.pi/4.-.5)
    if current_state == 2:
        p.setJointMotorControl2(jacoUid, 9, 
                        p.POSITION_CONTROL,math.pi/2. , force = 200)
        p.setJointMotorControl2(jacoUid, 11, 
                        p.POSITION_CONTROL, math.pi/2., force = 200)
        p.setJointMotorControl2(jacoUid, 13, 
                        p.POSITION_CONTROL, math.pi/2., force = 200)
    if current_state == 3:
        p.setJointMotorControl2(jacoUid, 2, 
                        p.POSITION_CONTROL,math.pi/2.+.5)
        p.setJointMotorControl2(jacoUid, 4, 
                        p.POSITION_CONTROL,4*math.pi/4.+.5)

    if state_t >state_durations[current_state]:
        current_state += 1
        print(current_state)
        if current_state >= len(state_durations):
            current_state = 0
        state_t = 0
    p.stepSimulation()

