import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)  # Set gravity to zero
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)  # Enable mouse interaction
useCollisionShapeQuery = False  # Changed to False to use current body positions
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
geom = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
geomBox = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.2, 0.2])
baseOrientationB = p.getQuaternionFromEuler([0, 0, 0])  # No initial rotation
basePositionB = [1.5, 0, 1]
obA = -1
obB = -1

obA = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=geom, basePosition=[0.5, 0, 1])
obB = p.createMultiBody(baseMass=1,  # Give mass to make it dynamic for mouse interaction
                        baseCollisionShapeIndex=geomBox,
                        basePosition=basePositionB,
                        baseOrientation=baseOrientationB)

lineWidth = 3
colorRGB = [1, 0, 0]
lineId = p.addUserDebugLine(lineFromXYZ=[0, 0, 0],
                            lineToXYZ=[0, 0, 0],
                            lineColorRGB=colorRGB,
                            lineWidth=lineWidth,
                            lifeTime=0)
# Removed rotation variables and loop updates

while (p.isConnected()):
  # Removed rotation code
  # The cube will stay static now

  if (useCollisionShapeQuery):
    pts = p.getClosestPoints(bodyA=-1,
                             bodyB=-1,
                             distance=100,
                             collisionShapeA=geom,
                             collisionShapeB=geomBox,
                             collisionShapePositionA=[0.5, 0, 1],
                             collisionShapePositionB=basePositionB,
                             collisionShapeOrientationB=baseOrientationB)
    #pts = p.getClosestPoints(bodyA=obA, bodyB=-1, distance=100, collisionShapeB=geomBox, collisionShapePositionB=basePositionB, collisionShapeOrientationB=baseOrientationB)
  else:
    pts = p.getClosestPoints(bodyA=obA, bodyB=obB, distance=100)

  if len(pts) > 0:
    #print(pts)
    distance = pts[0][8]
    #print("distance=",distance)
    ptA = pts[0][5]
    ptB = pts[0][6]
    p.addUserDebugLine(lineFromXYZ=ptA,
                       lineToXYZ=ptB,
                       lineColorRGB=colorRGB,
                       lineWidth=lineWidth,
                       lifeTime=0,
                       replaceItemUniqueId=lineId)
  p.stepSimulation()  # Step the physics simulation
  time.sleep(1./240.)  # Added step simulation for proper physics, though gravity is 0

#removeCollisionShape is optional:
#only use removeCollisionShape if the collision shape is not used to create a body
#and if you want to keep on creating new collision shapes for different queries (not recommended)
p.removeCollisionShape(geom)
p.removeCollisionShape(geomBox)
