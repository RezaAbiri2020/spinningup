
# this is the beginning of code for creating jaco for gym

# imports for gym
import gym
from gym import error, spaces, utils
from gym.utils import seeding

# imports for pybullet
import os
import pybullet as p
import pybullet_data
import math 
import numpy as np
import random



class JacoEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):

            p.connect(p.GUI)
            p.resetDebugVisualizerCamera(cameraDistance=2.3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])
            self.action_space = spaces.Box(np.array([-1]*3), np.array([1]*3))
            self.observation_space = spaces.Box(np.array([-1]*3), np.array([1]*3))

    def step(self,action):

        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        # define/force the orientation of end-effector to be down for picking up objects 
        orientation = p.getQuaternionFromEuler([0.,-math.pi,math.pi/2.])
        dv = 0.005
        dx = action[0] * dv
        dy = action[1] * dv
        dz = action[2] * dv
        
        # reading the end-effector (link 8) info: positions and orientations; only use positions
        currentPose = p.getLinkState(self.jacoUid, 8)
        currentPosition = currentPose[0]
        newPosition = [currentPosition[0] + dx,
                       currentPosition[1] + dy,
                       currentPosition[2] + dz]
        
        # calculate the inverse kin caused by pos and orientation for later necessary actuating 
        jointPoses = p.calculateInverseKinematics(self.jacoUid,8,newPosition, orientation)
        #print('necessary jointPoses for 7 links and 3 fingers are:')
        #print(jointPoses)

        # jointposes will be the values of all 7 joints and 3 fingers
        # move the robot a little bit
        p.setJointMotorControlArray(self.jacoUid, [1, 2, 3, 4, 5, 6, 7, 9, 11, 13], p.POSITION_CONTROL, list(jointPoses))

        p.stepSimulation()

        #state_object, _ = p.getBasePositionAndOrientation(self.objectUid)
        state_robot = p.getLinkState(self.jacoUid, 8)[0]
        #state_fingers = (p.getJointState(self.pandaUid,9)[0], p.getJointState(self.pandaUid, 10)[0])
        
        # assuming a target location for reaching in space with these positions in xyz
        target = [0.65, 0, 0.5]
        if abs(state_robot[0]-target[0])<0.05 and abs(state_robot[1]-target[1])<0.05 and abs(state_robot[2]-target[2])<0.05:
            reward = 1
            done = True
        else:
            reward = 0
            done = False
        info = state_robot
        observation = state_robot
        
        return observation, reward, done, info


    def reset(self):

        p.resetSimulation()
        # we will enable rendering after we loaded everything
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) 
        
        #p.setGravity(0,0,-10)
        urdfRootPath=pybullet_data.getDataPath()

        planeUid = p.loadURDF(os.path.join(urdfRootPath,"plane.urdf"), basePosition=[0,0,-0.65])

        # for fixed values 
        #rest_poses = [math.pi/1., math.pi/1., math.pi/1., math.pi/1., math.pi/1., math.pi/1., math.pi/1.]
        self.jacoUid = p.loadURDF(os.path.join(urdfRootPath,"jaco/j2s7s300_gym.urdf"), useFixedBase=True)
        
        # start with a random initial positions
        for i in range(7):
            p.resetJointState(self.jacoUid,i, random.uniform(1,3))

        tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])

        trayUid = p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"),basePosition=[0.65,0,0])

        #state_object= [random.uniform(0.5,0.8),random.uniform(-0.2,0.2),0.05]
        
        self.objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=[0.65,0,0.5])
        
        # get only the position of end-effector
        state_robot = p.getLinkState(self.jacoUid, 8)[0]

        # get the state of fingers for jaco 
        state_fingers = (p.getJointState(self.jacoUid,9)[0], p.getJointState(self.jacoUid, 11)[0], p.getJointState(self.jacoUid,13)[0])
        
        # for step one; only output the end-effector position
        observation = state_robot

        # rendering's back on again
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) 
        return observation


    def render(self, mode='human'):

        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7,0,0.05],
                                                            distance=.7,
                                                            yaw=90,
                                                            pitch=-70,
                                                            roll=0,
                                                            upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                     aspect=float(960) /720,
                                                     nearVal=0.1,
                                                     farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                              height=720,
                                              viewMatrix=view_matrix,
                                              projectionMatrix=proj_matrix,
                                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720,960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):

        p.disconnect()




