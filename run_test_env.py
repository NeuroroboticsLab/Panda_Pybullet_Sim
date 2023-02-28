from sim_world import Sim_World
import pybullet as p
from pybullet_utils import bullet_client

render = True

client = bullet_client.BulletClient(p.GUI if render else p.DIRECT)

env = Sim_World("test", client=client)

env.reset_env()

# Geh zu dieser run funktion und schreib dort alles rein was der roboter machen soll
env.run_env()
