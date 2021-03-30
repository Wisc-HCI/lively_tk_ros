import pybullet

# physicsClient = pybullet.connect(pybullet.GUI)
physicsClient = pybullet.connect(pybullet.GUI)

pybullet.setAdditionalSearchPath('/Users/schoen/ROS/ros2_lik/src/lively_ik/lively_ik_gui/public/assets')
# pybullet.setAdditionalSearchPath('/Users/schoen/ROS/ros2_lik/src/lively_ik/lively_ik/lively_ik/assets/urdf')

mesh = pybullet.createVisualShape(
    shapeType=pybullet.GEOM_MESH,
    fileName='moveit_resources_panda_description/meshes/collision/link2.dae')

while (pybullet.isConnected()):
  pybullet.stepSimulation()
  time.sleep(0.01) 
