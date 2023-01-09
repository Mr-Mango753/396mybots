import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1
x = 0
y = 0
z = .5
# for x in range(5):
#     for y in range(5):
#         for i in range(1,11):
#             pyrosim.Send_Cube(name="Box", pos=[x,y,i] , size=[length-(i*.1), width-(i*.1), height-(i*.1)])

# pyrosim.Start_SDF("world.urdf")
# pyrosim.Send_Cube(name="Torso", pos=[0,0,.5] , size=[length, width, height])
# pyrosim.Send_Joint( name = "Torso_Leg" , parent= "Torso" , child = "Leg" , type = "revolute", position = [.5,0,1])
# pyrosim.Send_Cube(name="Leg", pos=[1,0,1.5] , size=[length, width, height])
# pyrosim.End()




def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Leg", pos=[1,0,1.5] , size=[length, width, height])
    pyrosim.End()
    
def Create_Robot():
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Backleg", pos=[0,0,1] , size=[length, width, height])
    pyrosim.Send_Joint( name = "Torso_Backleg" , parent= "Torso" , child = "Backleg" , type = "revolute", position = [1,0,.5])
    pyrosim.Send_Cube(name="Torso", pos=[0,0,.5] , size=[length, width, height])
    pyrosim.Send_Joint( name = "Torso_Frontleg" , parent= "Torso" , child = "Frontleg" , type = "revolute", position = [1,0,.5])
    pyrosim.Send_Cube(name="Frontleg", pos=[1,0,0] , size=[length, width, height])
    pyrosim.End()

Create_World()
Create_Robot()

