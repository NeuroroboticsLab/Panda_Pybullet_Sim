from sim_world import Sim_World
import pybullet as p
from pybullet_utils import bullet_client
import numpy as np

RENDER = True


if __name__ == '__main__':
    client = bullet_client.BulletClient(p.GUI if RENDER else p.DIRECT)

    env = Sim_World("test", client=client)

    env.reset_env()

    # go to the run function and write there everything the robot should do

    env.visualize_target(env.get_pos_of_stone(3))
    for _ in range(20):
        p.stepSimulation()

    pos_of_stone = env.get_pos_of_stone(3)
    get_ori_of_stone = env.get_ori_of_stone(3)
    env.robot.move_stick(pos_of_stone)

    print("get_pos_of_stone", pos_of_stone)
    print("stick_pos", env.robot.get_stick_pos())

    # Drive robot in y direction for 100 steps
    # for _ in range(100):
    #     env.robot.move_tcp_delta([0, 0.05, 0])

    # # Drive robot in x direction for 100 steps
    # for _ in range(100):
    #     env.robot.move_tcp_delta([0.05, 0, 0])

    # self.robot.move_tcp_delta(new_pos, new_ori)

    for _ in range(100):
        p.stepSimulation()
    env.visualize_robot_joints()
    # Just run simulation without executing anything else
    for _ in range(1000000000):
        p.stepSimulation()
