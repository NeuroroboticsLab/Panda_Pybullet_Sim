import os, inspect, time
import numpy as np
import pybullet as p

from Panda import Panda

class Sim_World():

    def __init__(self, config, client):

        # Environment parameters
        self.config = config
        self.urdf_root = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "/models"
        self.robot = Panda(client, config=config, time_step=1. / 240.)
        self.jenga_stone_ids = []


    
    def reset_env(self):

        # Reset Simulation environment
        p.resetSimulation()
        p.setPhysicsEngineParameter(numSubSteps=5)
        p.setTimeStep(1/350)
        #p.setTimeStep(1/480.)
        p.setGravity(0, 0, -9.81)

        # Load table 
        p.loadURDF(os.path.join(self.urdf_root, "table/table.urdf"), 0.3, 0.0, -0.63)

        # Load robot model
        self.robot.reset_robot()

        self.build_jenga_tower()

        #TODO Place the jenga stones

    def build_jenga_tower(self):

        ori = p.getQuaternionFromEuler([0,0,1.57])


        for i in range(18):
            layer = []
            for j in range(3):
                if i % 2 == 0:
                    layer.append(p.loadURDF(os.path.join(self.urdf_root, "jenga/jenga.urdf"), 0.5, j*0.0265, (0.015*i)))
                else:
                    layer.append(p.loadURDF(os.path.join(self.urdf_root, "jenga/jenga.urdf"), 0.474+(j*0.0265), 0.0251, 0.015*i,ori[0],ori[1],ori[2],ori[3]))
            
            self.jenga_stone_ids.append(layer)

    def get_pos_of_stone(self, block_id):
        return p.getBasePositionAndOrientation(block_id)[0]
    
    def get_ori_of_stone(self, block_id):
        return p.getBasePositionAndOrientation(block_id)[1]
    
    def run_env(self):

        # Example functions
        # Drive robot in y direction for 100 steps
        for _ in range(20):
            self.robot.move_robot([0,0.05,0])

        # Drive robot in x direction for 100 steps
        for _ in range(20):
            self.robot.move_robot([0.05,0,0])

        # Example of accessing  a specific stone
        # e.g. the middle stone of layer 7 -> self.jenga_stone_ids[6][1]
        # the order is the following from left to right and from back to front
        # e.g. the left stone in the first layer -> self.jenga_stone_ids[0][0]
        # e.g. the front stone in the second layer -> self.jenga_stone_ids[1][2]
        pos = self.get_pos_of_stone(self.jenga_stone_ids[1][2])
        ori = self.get_ori_of_stone(self.jenga_stone_ids[1][2])

        # Just run simulation without executing anything else
        for _ in range(1000000000):
            p.stepSimulation()
        


        

        






    
  
        

