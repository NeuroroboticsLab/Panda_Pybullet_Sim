from sim_world import Sim_World
import pybullet as p
import numpy as np
from pybullet_utils import bullet_client

RENDER = True


if __name__ == '__main__':
    client = bullet_client.BulletClient(p.GUI if RENDER else p.DIRECT)

    num_layers = 10 # 18
    env = Sim_World("test", client=client, num_layers=num_layers)

    env.reset_env()

    stone_id = 12
    env.visualize_target(env.get_pos_of_stone(stone_id))
    for _ in range(20):
        p.stepSimulation()

    pos_of_stone = env.get_pos_of_stone(stone_id)
    get_ori_of_stone = env.get_ori_of_stone(stone_id)
    env.robot.move_stick(pos_of_stone)
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

