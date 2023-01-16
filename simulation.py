import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
# p.loadSDF("world.sdf")
robotId = p.loadURDF("body.urdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = numpy.zeros(100)
frontLegSensorValues = numpy.zeros(100)
for i in range(1000):
    # backLegTouch = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    # print(backLegTouch)
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    p.stepSimulation()
    time.sleep(1/60)
    print(backLegSensorValues)
    numpy.save("backLegSensorValues", backLegSensorValues, allow_pickle=True, fix_imports=True)
    numpy.save("frontLegSensorValues", frontLegSensorValues, allow_pickle=True, fix_imports=True)
p.disconnect()
