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

    def __init__(self) -> None:
        self.weights = numpy.array([[numpy.random.rand(), numpy.random.rand()],
                                    [numpy.random.rand(), numpy.random.rand()],
                                    [numpy.random.rand(), numpy.random.rand()]])
        self.weights = self.weights * 2 - 1
        self.fitness = 0
        self.hello = "hello"

    def Evaluate(self, method):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()
        f = open("fitness.txt", "r")
        self.fitness = float(f.read())
        f.close()
        os.system("python simulation.py " + method)

        
    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[0,0.5,0.5] , size=[length, width, height])
        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="BackLeg", pos=[.5,0,-.5] , size=[length, width, height])
        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [2,0,1])
        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5] , size=[length, width, height])
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [1,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[-.5,0,-.5] , size=[length, width, height])
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain.nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")    
        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")
        # pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 2 , weight = .1)
        # pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 2 , weight = .1)
        # pyrosim.Send_Synapse( sourceNeuronName = 0 , targetNeuronName = 3 , weight = .1)
        # pyrosim.Send_Synapse( sourceNeuronName = 1 , targetNeuronName = 3 , weight = -.1)
        # pyrosim.Send_Synapse( sourceNeuronName = 2 , targetNeuronName = 4 , weight = -.2)

        for currentRow in [0,1,2]:
            for currentColumn in [0,1]:
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+3, weight=self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        randRow = random.randint(0,2)
        randColumn = random.randint(0,1)
        self.weights[randRow, randColumn] = random.random() * 2 - 1
