import os
import inspect
import time
import numpy as np
import pybullet as p

from Panda import Panda
from Panda import Q_INIT


class Sim_World():

    def __init__(self, config, client):
        # Environment parameters
        self.config = config
        self.urdf_root = os.path.dirname(os.path.abspath(
            inspect.getfile(inspect.currentframe()))) + "/models"
        self.robot = Panda(client, config=config, time_step=1. / 240.)
        self.jenga_stone_ids = []

    def reset_env(self):

        # Reset Simulation environment
        p.resetSimulation()
        p.setPhysicsEngineParameter(numSubSteps=5)
        p.setTimeStep(1/350)
        # p.setTimeStep(1/480.)
        p.setGravity(0, 0, -9.81)

        # Load table
        p.loadURDF(os.path.join(self.urdf_root,
                   "table/table.urdf"), 0.3, 0.0, -0.63)

        # Load robot model
        self.robot.reset_robot(Q_INIT.RIGHT)

        self.build_jenga_tower(5)

    def build_jenga_tower(self, num_layers=18, pos=[0.5, 0, 0]):
        stone_path = os.path.join(self.urdf_root, "jenga/block.urdf")

        ori = p.getQuaternionFromEuler([0, 0, 1.57])

        for layer_index in range(num_layers):
            layer = []
            for stone_index in range(3):
                if layer_index % 2 == 0:
                    layer.append(p.loadURDF(
                        stone_path, pos[0], pos[1] + stone_index*0.0265, pos[2] + (0.015*layer_index)))
                else:
                    layer.append(p.loadURDF(stone_path, pos[0] - 0.0265 + (
                        stone_index*0.0265), pos[1] + 0.0251, pos[2] + 0.015*layer_index, ori[0], ori[1], ori[2], ori[3]))

            self.jenga_stone_ids.append(layer)

    def get_pos_of_stone(self, block_id):
        return p.getBasePositionAndOrientation(block_id)[0]

    def get_ori_of_stone(self, block_id):
        return p.getBasePositionAndOrientation(block_id)[1]

    def visualize_target(self, target_pos):
        sphere_path = os.path.join(self.urdf_root, "sphere.urdf")
        marker_id = p.loadURDF(sphere_path,
                               target_pos[0], target_pos[1], target_pos[2], 0.000000, 0.000000, 0.0, 1.0)
        p.setCollisionFilterGroupMask(marker_id, 0, 0, 0)

    def visualize_robot_joints(self):
        for joint_index in range(9, 13):
            joint_info = p.getLinkState(self.robot.robot_id, joint_index)
            joint_pos = joint_info[0]
            print(joint_info)
            self.visualize_target(joint_pos)

    def run_env(self):

        self.visualize_target([0.3, 0.3, 0.3])

        # Drive robot in y direction for 100 steps
        for _ in range(100):
            self.robot.move_robot([0, 0.05, 0])

        # Drive robot in x direction for 100 steps
        for _ in range(100):
            self.robot.move_robot([0.05, 0, 0])

        new_pos, new_ori = self.robot.trans_stick_to_eef(
            [0.3, 0.3, 0.3], [-0.7, np.pi, 0])
        # self.robot.move_robot(new_pos, new_ori)

        for _ in range(20):
            p.stepSimulation()
        # Just run simulation without executing anything else
        for _ in range(1000000000):
            p.stepSimulation()
