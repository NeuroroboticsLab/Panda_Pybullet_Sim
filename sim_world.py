import os
import inspect
import math
import numpy as np
import pybullet as p

from Panda import Panda
from Panda import Q_INIT
from Panda import ROBOT_TYPE


class Sim_World():

    def __init__(self, config, client, num_layers=18):
        # Environment parameters
        self.config = config
        self.urdf_root = os.path.dirname(os.path.abspath(
            inspect.getfile(inspect.currentframe()))) + "/models"
        self.robot = Panda(client, config=config, time_step=1. / 240.)
        self.jenga_stone_ids = []
        self.num_layers = num_layers
        self.init_robot_pose = Q_INIT.LEFT
        self.robot_type = ROBOT_TYPE.STICK_LEFT_TOP

    def reset_env(self):
        # Reset Simulation environment
        p.resetSimulation()
        p.setPhysicsEngineParameter(numSubSteps=5)
        p.setTimeStep(1/350)  # p.setTimeStep(1/480.)
        p.setGravity(0, 0, -9.81)

        # Load table
        p.loadURDF(os.path.join(self.urdf_root,
                   "table/table.urdf"), 0.3, 0.0, -0.63)

        # Load robot model
        self.robot.reset_robot(robot_type=self.robot_type,
                               init_state=self.init_robot_pose)

        self.build_jenga_tower(self.num_layers, pos=[
                               0.6, 0.1, 0], rot_z=0)  # rot_z=math.pi/4

    def build_jenga_tower(self, num_layers, pos=[0.5, 0, 0], rot_z=0):
        stone_path = os.path.join(self.urdf_root, "jenga/block.urdf")

        ori_1 = p.getQuaternionFromEuler([0, 0, rot_z])
        ori_2 = p.getQuaternionFromEuler([0, 0, rot_z + math.pi/2])

        r_1 = np.array(p.getMatrixFromQuaternion(
            ori_1), dtype=np.float64).reshape(3, 3)
        r_2 = np.array(p.getMatrixFromQuaternion(
            ori_2), dtype=np.float64).reshape(3, 3)

        for layer_index in range(num_layers):
            layer = []
            for stone_index in range(3):
                diff = np.array([0, 0.0265, 0], dtype=np.float64)

                if layer_index % 2 == 0:
                    diff = r_1.dot(diff)
                    stone_p = pos + diff * (stone_index - 1)
                    stone_p[2] += 0.015*layer_index

                    layer.append(p.loadURDF(
                        stone_path, stone_p[0], stone_p[1], stone_p[2], ori_1[0], ori_1[1], ori_1[2], ori_1[3]))
                else:
                    diff = r_2.dot(diff)
                    stone_p = pos + diff * (stone_index - 1)
                    stone_p[2] += 0.015*layer_index

                    layer.append(p.loadURDF(
                        stone_path, stone_p[0], stone_p[1], stone_p[2], ori_2[0], ori_2[1], ori_2[2], ori_2[3]))

            self.jenga_stone_ids.append(layer)

    def get_pos_of_stone(self, block_id):
        return p.getBasePositionAndOrientation(block_id)[0]

    def get_ori_of_stone(self, block_id):
        quat = p.getBasePositionAndOrientation(block_id)[1]
        return np.array(p.getEulerFromQuaternion(quat), dtype=np.float32)

    def visualize_target(self, target_pos):
        sphere_path = os.path.join(self.urdf_root, "sphere.urdf")
        marker_id = p.loadURDF(
            sphere_path, target_pos[0], target_pos[1], target_pos[2], 0.0, 0.0, 0.0, 1.0)
        p.setCollisionFilterGroupMask(marker_id, 0, 0, 0)

    def visualize_robot_joints(self):
        # print red dots ant the tcp, fingers and stick
        for joint_index in range(9, 14):
            joint_info = p.getLinkState(self.robot.robot_id, joint_index)
            joint_pos = joint_info[0]
            print(joint_info)
            self.visualize_target(joint_pos)

    def run_env(self):
        # Example functions
        # Drive robot in y direction for 100 steps
        self.robot.move_tcp_delta([0, 0.05, 0])

        # Drive robot in x direction for 100 steps
        self.robot.move_tcp_delta([0.05, 0, 0])

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
