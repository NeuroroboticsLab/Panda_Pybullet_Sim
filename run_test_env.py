from sim_world import Sim_World
import pybullet as p
from pybullet_utils import bullet_client

RENDER = True


if __name__ == '__main__':
    client = bullet_client.BulletClient(p.GUI if RENDER else p.DIRECT)

    env = Sim_World("test", client=client)

    env.reset_env()

    # go to the run function and write there everything the robot should do
    env.run_env()

