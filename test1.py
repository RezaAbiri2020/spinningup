# a test for jaco in pybullet

import os
import pybullet as p
import pybullet_data

p.connect(p.GUI)
pandaUid = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"franka_panda/panda.urdf"), useFixedBase=True)

while True:
    p.stepSimulation()
