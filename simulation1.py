from world import WORLD
from robot import ROBOT
from sensor import SENSOR
import time 
import pybullet as p
import pybullet_data 
import pyrosim.pyrosim as pyrosim
import constants as c

class SIMULATION:

    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        self.world = WORLD()
        self.robot = ROBOT()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        self.planeId = p.loadURDF("plane.urdf")
        p.loadSDF("world.sdf")

    def Run(self):
        for i in range(c.vectorSize):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Act(i)
            time.sleep(1/60)

    def __del__(self):
        p.disconnect()