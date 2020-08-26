# a test for jaco in pybullet

import os
import pybullet as p
import pybullet_data

p.connect(p.GUI)
pandaUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"jaco/j2s7s300_gym.urdf"), useFixedBase=True)

# record a video
p.setRealTimeSimulation(0)
p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, 'Video11.MP4')

while True:
    p.stepSimulation()
