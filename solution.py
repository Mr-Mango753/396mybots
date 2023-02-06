from world import WORLD
from robot import ROBOT
from sensor import SENSOR
import time 
import pybullet as p
import pybullet_data 
import pyrosim.pyrosim as pyrosim
import constants as c
import numpy
import random
import os

length = 1
width = 1
height = 1
x = 0
y = 0
z = .5

class SOLUTION:

    def __init__(self, ID) -> None:
        # self.weights = numpy.array([[numpy.random.rand(), numpy.random.rand()],
        #                             [numpy.random.rand(), numpy.random.rand()],
        #                             [numpy.random.rand(), numpy.random.rand()]])
        # self.weights = self.weights * 2 - 1
        self.weights = numpy.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights = self.weights*2-1
        # self.fitness = 0
        self.hello = "hello"
        self.myID = ID

    def Set_ID(self):
        self.myID += 1

    # def Evaluate(self, directOrGUI):
    #     self.Create_World()
    #     self.Generate_Body()
    #     self.Generate_Brain()
    #     # os.system("python simulation.py " + directOrGUI + " " + str(self.myID) + " &")
    #     while not os.path.exists("fitness"+str(self.myID)+".txt"):
    #         time.sleep(.01)
    #     f = open("fitness"+str(self.myID)+".txt", "r")
    #     self.fitness = float(f.read())
    #     f.close()
    #     os.system("start /B " + "python simulation.py " + directOrGUI + " " + str(self.myID))
        
    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        if self.myID == 0:
            self.Generate_Body()
        self.Generate_Brain()
        os.system("start /B " + "python simulation.py " + directOrGUI + " " + str(self.myID))
        # os.system("python simulation.py " + directOrGUI + " " + str(self.myID) + " &")


    def Wait_For_Simulation_To_End(self):
        while not os.path.exists(f"fitness{str(self.myID)}.txt"):
            time.sleep(0.01)
        f = open(f"fitness{self.myID}.txt", "r")
        self.fitness = float(f.read())
        f.close()
        os.system(f"del fitness{str(self.myID)}.txt")
        
    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[2,5,2] , size=[2, 2, 2])
        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        # pyrosim.Send_Cube(name="BackLeg", pos=[-.5,0,-.5] , size=[length, width, height])
        # pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
        # pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[length, width, height])
        # pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
        # pyrosim.Send_Cube(name="FrontLeg", pos=[.5,0,-.5] , size=[length, width, height])
        
        # quadruped stuff
        # pyrosim.Send_Cube(name="Torso", pos=[0,0,1], size=[1,1,1])

        # pyrosim.Send_Joint( name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0,-0.5,1], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0], size=[0.2,-1,0.2])
        # pyrosim.Send_Joint( name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="BackLowerLeg", pos=[0,0,-0.5], size=[0.2,0.2,1])

        # pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0,0.5,1], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0], size=[0.2,1,0.2])
        # pyrosim.Send_Joint( name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [0,1,0], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0,0,-0.5], size=[0.2,0.2,1])
        
        # pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5,0,1], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0], size=[1,0.2,0.2])
        # pyrosim.Send_Joint( name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-0.5], size=[0.2,0.2,1])
        
        # pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5,0,1], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0], size=[1,0.2,0.2])
        # pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.5], size=[0.2,0.2,1])

        # Final project stuff
        pyrosim.Send_Cube(name="Torso", pos=[0,0,1], size=[1,.5,.05])

        # pyrosim.Send_Joint( name = "Torso_BackLeg",parent="Torso",child="BackLeg",type="revolute",position=[-.1,-.1,1.05], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0], size=[0.05,-1,0.05])
        # pyrosim.Send_Joint( name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [0,-1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="BackLowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])

        # pyrosim.Send_Joint( name = "Torso_LeftLeg",parent="Torso",child="LeftLeg",type="revolute",position=[.1,-.1,1.05], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="LeftLeg", pos=[0,-0.5,0], size=[0.05,-1,0.05])
        # pyrosim.Send_Joint( name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [0,-1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])

        # pyrosim.Send_Joint( name = "Torso_Front4Leg",parent="Torso",child="Front4Leg",type="revolute",position=[.4,-.1,1.05], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="Front4Leg", pos=[0,-0.5,0], size=[0.05,-1,0.05])
        # pyrosim.Send_Joint( name = "Front4Leg_Front4LowerLeg" , parent= "Front4Leg" , child = "Front4LowerLeg" , type = "revolute", position = [0,-1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="Front4LowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])

        # pyrosim.Send_Joint( name = "Torso_Front5Leg",parent="Torso",child="Front5Leg",type="revolute",position=[-.4,-.1,1.05], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="Front5Leg", pos=[0,-0.5,0], size=[0.05,-1,0.05])
        # pyrosim.Send_Joint( name = "Front5Leg_Front5LowerLeg" , parent= "Front5Leg" , child = "Front5LowerLeg" , type = "revolute", position = [0,-1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="Front5LowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])




        pyrosim.Send_Joint( name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [-.1,.1,1.05], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0], size=[0.05,1,0.05])
        # pyrosim.Send_Joint( name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [0,1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])

        pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [.1,.1,1.05], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0,0.5,0], size=[0.05,1,0.05])
        # pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [0,1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])

        pyrosim.Send_Joint( name = "Torso_Front2Leg" , parent= "Torso" , child = "Front2Leg" , type = "revolute", position = [-.4,.1,1.05], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="Front2Leg", pos=[0,0.5,0], size=[0.05,1,0.05])
        # pyrosim.Send_Joint( name = "Front2Leg_Front2LowerLeg" , parent= "Front2Leg" , child = "Front2LowerLeg" , type = "revolute", position = [0,1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="Front2LowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])

        pyrosim.Send_Joint( name = "Torso_Front3Leg" , parent= "Torso" , child = "Front3Leg" , type = "revolute", position = [.4,.1,1.05], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="Front3Leg", pos=[0,0.5,0], size=[0.05,1,0.05])
        # pyrosim.Send_Joint( name = "Front3Leg_Front3LowerLeg" , parent= "Front3Leg" , child = "Front3LowerLeg" , type = "revolute", position = [0,1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="Front3LowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])

        pyrosim.Send_Joint( name = "Torso_Front4Leg" , parent= "Torso" , child = "Front4Leg" , type = "revolute", position = [.5,.06,.02], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="Front4Leg", pos=[0,0,1], size=[.05,.05,.10])
        # pyrosim.Send_Joint( name = "Front3Leg_Front3LowerLeg" , parent= "Front3Leg" , child = "Front3LowerLeg" , type = "revolute", position = [0.7,.05,.45], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="Front3LowerLeg", pos=[-.07,-0.05,-0.5], size=[.05,.05,.2])


        # pyrosim.Send_Joint( name = "Torso_TorsoTail" , parent= "Torso" , child = "TorsoTail" , type = "revolute", position = [.4,.1,1.15], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="TorsoTail", pos=[0,0.5,0], size=[0.05,1,0.05])
        # pyrosim.Send_Joint( name = "TorsoTail_TailLeg" , parent= "TorsoTail" , child = "TailLeg" , type = "revolute", position = [1,1,.4], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="TailLeg", pos=[0,0,-0.5], size=[0.1,0.1,.3])


        # pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [.1,.1,1.15], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="RightLeg", pos=[0,0.5,0], size=[0.1,1,0.1])
        # pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [0,1,.2], jointAxis = "1 0 0")
        # pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.5])
        
        # pyrosim.Send_Joint( name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5,0,1], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0], size=[.1,0.1,0.1])
        # pyrosim.Send_Joint( name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.1])
        
        # pyrosim.Send_Joint( name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5,0,1], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0], size=[.1,0.1,0.1])
        # pyrosim.Send_Joint( name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
        # pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.5], size=[0.1,0.1,.1])
        
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        # pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontLeg")    
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "RightLeg")    
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "Front2Leg")    
        pyrosim.Send_Sensor_Neuron(name = 4 , linkName = "Front3Leg")    
        pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "Front4Leg")    
        # pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "Front3LowerLeg")    
        # pyrosim.Send_Sensor_Neuron(name = 5 , linkName = "Front4Leg")    
        # pyrosim.Send_Sensor_Neuron(name = 6 , linkName = "Front5Leg")    
        # pyrosim.Send_Motor_Neuron(name = 5 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 6, jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name = 7 , jointName = "Torso_Front2Leg")
        pyrosim.Send_Motor_Neuron( name = 8 , jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name = 9 , jointName = "Torso_Front3Leg")
        pyrosim.Send_Motor_Neuron(name = 10, jointName = "Torso_Front4Leg")
        # pyrosim.Send_Motor_Neuron(name = 11, jointName = "Front3Leg_Front3LowerLeg")
        # pyrosim.Send_Motor_Neuron(name = 11, jointName = "Torso_Front4Leg")
        # pyrosim.Send_Motor_Neuron(name = 12, jointName = "Torso_Front5Leg")
        # pyrosim.Send_Motor_Neuron(name = 13, jointName = "Torso_TorsoTail")


        # pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 2 , weight = .1)
        # pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 2 , weight = .1)
        # pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 3 , weight = .1)
        # pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 3 , weight = -.1)
        # pyrosim.Send_Synapse( sourceNeuronName = 2 , targetNeuronName = 4 , weight = -.2)

        # pyrosim.Send_Sensor_Neuron(name = 7 , linkName = "Torso")
        # pyrosim.Send_Sensor_Neuron(name = 10, linkName = "BackLeg")
        # pyrosim.Send_Sensor_Neuron(name = 8, linkName = "FrontLeg")
        # pyrosim.Send_Sensor_Neuron(name = 17, linkName = "BackLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name = 18, linkName = "FrontLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name = 19, linkName = "LeftLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name = 20, linkName = "RightLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name = 21, linkName = "LeftLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name = 22, linkName = "RightLowerLeg")

        # pyrosim.Send_Motor_Neuron( name = 12 , jointName = "Torso_BackLeg")
        # pyrosim.Send_Motor_Neuron( name = 14 , jointName = "Torso_LeftLeg")
        # pyrosim.Send_Motor_Neuron( name = 27 , jointName = "BackLeg_BackLowerLeg")
        # pyrosim.Send_Motor_Neuron( name = 28 , jointName = "FrontLeg_FrontLowerLeg")
        # pyrosim.Send_Motor_Neuron( name = 29 , jointName = "Front2Leg_Front2LowerLeg")
        # pyrosim.Send_Motor_Neuron( name = 30 , jointName = "Front3Leg_Front3LowerLeg")
        # pyrosim.Send_Motor_Neuron( name = 31 , jointName = "Front4Leg_Front4LowerLeg")
        # pyrosim.Send_Motor_Neuron( name = 32 , jointName = "Front5Leg_Front5LowerLeg")
        # pyrosim.Send_Motor_Neuron( name = 33 , jointName = "LeftLeg_LeftLowerLeg")
        # pyrosim.Send_Motor_Neuron( name = 34 , jointName = "RightLeg_RightLowerLeg")
        # pyrosim.Send_Motor_Neuron( name = 35 , jointName = "TorsoTail_TailLeg")

        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+c.numSensorNeurons, weight=self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Set_ID(self, ID):
        self.myID = ID

    def Mutate(self):
        randRow = random.randint(0,c.numSensorNeurons-1)
        randColumn = random.randint(0,c.numMotorNeurons-1)
        self.weights[randRow, randColumn] = random.random() * 2 - 1