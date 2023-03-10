import pybullet as p
import pybullet_data
import numpy as np
import math
from enum import Enum

class Q_init(Enum):
    LEFT = "left"
    RIGHT = "right"
    FRONT = "front"
    BACK = "back"
    

class Panda():

  def __init__(self, client, config , time_step=1./240):
    """
    Args:
      client: pybullet client
      config: env_params
      time_step: time to wait between simulation steps
    """

    self.client = client
    self.client.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    self.config = config

    self.robot_num_joints = 7
    self.robot_gripper_index = 11
    self.urdf_root_path = "franka_panda/panda.urdf"

    self.reset_robot()


  def reset_robot(self, init_state=Q_init.LEFT):
    # Load robot into simulation
    self.robot_id = self.client.loadURDF(self.urdf_root_path,basePosition=[0,0,0], useFixedBase=True)

    # Reset robot position and orientation
    self.client.resetBasePositionAndOrientation(self.robot_id, [-0.00000, 0.000000, 0.000000],
                                      [0.000000, 0.000000, 0.000000, 1.000000])

    # choose and initialize joint states
    if init_state is Q_init.LEFT:
      self.q_init_pos = [
          0.7, -0.7, 0, -2.5, 0, 1.57, 0, 0, 0, 0, 0, 0
      ]
    else:
      self.q_init_pos = [
          0, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0,
          0, 0.000000, 0, 0
      ]

    # Reset joint states to initial values 
    for q_index in range(len(self.q_init_pos)):
      self.client.resetJointState(self.robot_id, q_index, self.q_init_pos[q_index])

    for i in range(50):
      p.stepSimulation()

  def move_robot(self, delta_pos, ori_in=[0, math.pi, 0]):

    curr_tcp_pos = np.array(p.getLinkState(self.robot_id, self.robot_gripper_index)[0])

    new_tcp_pos = curr_tcp_pos + np.array(delta_pos,dtype=np.float32)

    ori = self.client.getQuaternionFromEuler(ori_in)

    # Get joint values for new eef position
    q_pos = self.client.calculateInverseKinematics(self.robot_id,
                                                    self.robot_gripper_index,
                                                    new_tcp_pos,
                                                    ori)

    # Move set joints to target position
    for q_idx in range(self.robot_num_joints):
      self.client.setJointMotorControl2(self.robot_id, q_idx, self.client.POSITION_CONTROL,
                                targetPosition=q_pos[q_idx])

    for _ in range(5):
      p.stepSimulation()

  def open_gripper(self):
    self.client.setJointMotorControlArray(self.robot_id, [9,10], self.client.POSITION_CONTROL, 
              targetPositions=[2,2])

    for _ in range(20):
      p.stepSimulation()


  def move_gripper(self, dist):

    tcp_finger_pos = self.get_joint_angles([9,10])

    diff = -(np.array(tcp_finger_pos) - np.array(dist/2))


    self.client.setJointMotorControlArray(self.robot_id, [9,10], self.client.POSITION_CONTROL, 
              targetPositions=diff)

    for _ in range(50):
      p.stepSimulation()


  def get_eef_pos(self):
    return np.array(p.getLinkState(self.robot_id, self.robot_gripper_index)[0])

  def get_joint_angles(self, joint_ids=[0,1,2,3,4,5,6]):
    curr_joint_states = p.getJointStates(self.robot_id, joint_ids)
    curr_joint_angles = []

    for state in curr_joint_states:
        curr_joint_angles.append(state[0])
    
    return curr_joint_angles

  def get_joint_velocities(self):
    curr_joint_states = p.getJointStates(self.robot_id, [0,1,2,3,4,5,6])
    curr_joint_velocities = []

    for state in curr_joint_states:
        curr_joint_velocities.append(state[1])
    
    return curr_joint_velocities

  def get_joint_torques(self):
    curr_joint_states = p.getJointStates(self.robot_id, [0,1,2,3,4,5,6])
    curr_joint_torques = []

    for state in curr_joint_states:
        curr_joint_torques.append(state[3])
    
    return curr_joint_torques

  

