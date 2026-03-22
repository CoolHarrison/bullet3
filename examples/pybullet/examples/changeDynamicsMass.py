import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
cube2 = p.loadURDF("cube.urdf", [0, 0, 3], useFixedBase=True)
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
timeStep = 1. / 240.
p.setTimeStep(timeStep)

#now cube2 will have a floating base and move
massID = p.addUserDebugParameter("mass", 0, 1, 1)

while (p.isConnected()):
  mass = p.readUserDebugParameter(massID)
  p.changeDynamics(cube2, -1, mass=mass)
  p.stepSimulation()
  time.sleep(timeStep)


