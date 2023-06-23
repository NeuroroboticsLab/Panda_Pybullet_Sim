from scipy.spatial.transform import Rotation
import os
import pybullet as p
import pybullet_data
import numpy as np
import math
from enum import Enum


class Q_INIT(Enum):
    LEFT = "left"
    RIGHT = "right"
    TOP = "top"
    FRONT = "front"
    BACK = "back"


class ROBOT_TYPE(Enum):
    PANDA = "panda"
    STICK_RIGHT_TOP = "panda_stick_rigth_top"
    STICK_RIGHT_BACK = "panda_stick_rigth_back"
    STICK_RIGHT_FRONT = "panda_stick_rigth_front"
    STICK_LEFT_TOP = "panda_stick_left_top"
    STICK_LEFT_BACK = "panda_stick_left_back"
    STICK_LEFT_FRONT = "panda_stick_left_front"


GRIPPER_STEPS = 50
MOVE_STEPS = 30
MOVE_STEPS_DELTA = 10


def quaternion_from_rotation_matrix(R):
    rotation = Rotation.from_matrix(R)
    quaternion = rotation.as_quat()
    return quaternion


class Panda():
    def __init__(self, client, config, time_step=1./240):
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
        self.robot_stick_index = 12
        self.robot_gripper_index = 13
        self.jd = [
            0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001, 0.00001,
            0.00001, 0.00001
        ]

        self.reset_robot()

    def reset_robot(self, robot_type=ROBOT_TYPE.STICK_RIGHT_TOP, init_state=Q_INIT.LEFT):
        # Load robot into simulation
        urdf_robot_path = self.get_robot_path(robot_type)
        q_init_pos = self.get_joints_from_state(init_state)
        self.robot_id = self.client.loadURDF(urdf_robot_path, basePosition=[
                                             0, 0, 0], useFixedBase=True)
        self.client.resetBasePositionAndOrientation(self.robot_id, [-0.00000, 0.000000, 0.000000],
                                                    [0.000000, 0.000000, 0.000000, 1.000000])

        # choose and initialize joint states
        for q_index in range(len(q_init_pos)):
            self.client.resetJointState(
                self.robot_id, q_index, q_init_pos[q_index])

        # print joint info
        for i in range(self.client.getNumJoints(self.robot_id)):
            print(self.client.getJointInfo(self.robot_id, i))

        for i in range(50):
            p.stepSimulation()

    def move_tcp_delta(self, delta_pos, delta_ori=[0, 0, 0], degrees=False):
        if degrees is True:
            target_ori = np.deg2rad(target_ori)

        new_tcp_pos = self.get_eef_pos() + np.array(delta_pos, dtype=np.float32)
        new_tcp_ori = self.get_eef_ori() + np.array(delta_ori, dtype=np.float32)
        quat = self.client.getQuaternionFromEuler(new_tcp_ori)

        # Get joint values for new eef position
        q_pos = self.client.calculateInverseKinematics(self.robot_id, self.robot_gripper_index,
                                                       new_tcp_pos, quat, jointDamping=self.jd)

        # Move set joints to target position
        for q_idx in range(self.robot_num_joints):
            self.client.setJointMotorControl2(self.robot_id, q_idx, self.client.POSITION_CONTROL,
                                              targetPosition=q_pos[q_idx])

        for _ in range(MOVE_STEPS_DELTA):
            p.stepSimulation()

    def move_tcp(self, target_pos, target_ori=[0, math.pi, 0], degrees=False):
        if degrees is True:
            target_ori = np.deg2rad(target_ori)

        if degrees is True:
            target_ori = np.deg2rad(target_ori)
        quat = self.client.getQuaternionFromEuler(target_ori)

        for _ in range(MOVE_STEPS):
            q_pos = self.client.calculateInverseKinematics(
                self.robot_id, self.robot_gripper_index, target_pos, quat, jointDamping=self.jd)
            for q_idx in range(self.robot_num_joints):
                self.client.setJointMotorControl2(
                    self.robot_id, q_idx, self.client.POSITION_CONTROL, targetPosition=q_pos[q_idx])
            for _ in range(MOVE_STEPS_DELTA):
                p.stepSimulation()

    def move_stick_delta(self, delta_pos, delta_ori=[0, 0, 0], degrees=False):
        if degrees is True:
            target_ori = np.deg2rad(target_ori)

        new_tcp_pos = self.get_stick_pos() + np.array(delta_pos, dtype=np.float32)
        new_tcp_ori = self.get_stick_ori() + np.array(delta_ori, dtype=np.float32)
        quat = self.client.getQuaternionFromEuler(new_tcp_ori)

        # Get joint values for new stick position
        q_pos = self.client.calculateInverseKinematics(self.robot_id, self.robot_stick_index,
                                                       new_tcp_pos, quat, jointDamping=self.jd)

        # Move set joints to target position
        for q_idx in range(self.robot_num_joints):
            self.client.setJointMotorControl2(self.robot_id, q_idx, self.client.POSITION_CONTROL,
                                              targetPosition=q_pos[q_idx])

        for _ in range(MOVE_STEPS_DELTA):
            p.stepSimulation()

    def move_stick(self, target_pos, target_ori=[0, math.pi, 0], degrees=False):
        quat = self.client.getQuaternionFromEuler(target_ori)

        for _ in range(MOVE_STEPS):
            q_pos = self.client.calculateInverseKinematics(
                self.robot_id, self.robot_stick_index, target_pos, quat, jointDamping=self.jd)
            for q_idx in range(self.robot_num_joints):
                self.client.setJointMotorControl2(
                    self.robot_id, q_idx, self.client.POSITION_CONTROL, targetPosition=q_pos[q_idx])
            for _ in range(MOVE_STEPS_DELTA):
                p.stepSimulation()

    def open_gripper(self):
        self.client.setJointMotorControlArray(self.robot_id, [9, 10], self.client.POSITION_CONTROL,
                                              targetPositions=[2, 2])
        for _ in range(GRIPPER_STEPS):
            p.stepSimulation()

    def move_gripper(self, dist):
        tcp_finger_pos = self.get_joint_angles([9, 10])

        diff = -(np.array(tcp_finger_pos) - np.array(dist/2))

        self.client.setJointMotorControlArray(self.robot_id, [9, 10], self.client.POSITION_CONTROL,
                                              targetPositions=diff)

        for _ in range(GRIPPER_STEPS):
            p.stepSimulation()

    def get_eef_pos(self):
        return np.array(p.getLinkState(self.robot_id, self.robot_gripper_index)[0])

    def get_eef_ori(self):
        quat = np.array(p.getLinkState(
            self.robot_id, self.robot_gripper_index)[1])
        return np.array(self.client.getEulerFromQuaternion(quat), dtype=np.float32)

    def get_stick_pos(self):
        return np.array(p.getLinkState(self.robot_id, self.robot_stick_index)[0])

    def get_stick_ori(self):
        quat = np.array(p.getLinkState(
            self.robot_id, self.robot_stick_index)[1])
        return np.array(self.client.getEulerFromQuaternion(quat), dtype=np.float32)

    def get_joint_angles(self, joint_ids=[0, 1, 2, 3, 4, 5, 6]):
        curr_joint_states = p.getJointStates(self.robot_id, joint_ids)
        curr_joint_angles = []

        for state in curr_joint_states:
            curr_joint_angles.append(state[0])

        return curr_joint_angles

    def get_joint_velocities(self):
        curr_joint_states = p.getJointStates(
            self.robot_id, [0, 1, 2, 3, 4, 5, 6])
        curr_joint_velocities = []

        for state in curr_joint_states:
            curr_joint_velocities.append(state[1])

        return curr_joint_velocities

    def get_joint_torques(self):
        curr_joint_states = p.getJointStates(
            self.robot_id, [0, 1, 2, 3, 4, 5, 6])
        curr_joint_torques = []

        for state in curr_joint_states:
            curr_joint_torques.append(state[3])

        return curr_joint_torques

    def get_joints_from_state(self, state):
        if state is Q_INIT.LEFT:
            return [0.7, -0.7, 0, -2.5, 0, 1.57, 0, 0, 0, 0, 0, 0]
        elif state is Q_INIT.RIGHT:
            return [-0.7, -0.7, 0, -2.5, 0, 1.57, 0, 0, 0, 0, 0, 0]

        return [0, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0, 0, 0, 0, 0]

    def get_robot_path(self, robot_type):
        urdf_root = os.getcwd() + "/models/franka_panda/"
        urdf_robot_path = urdf_root + "panda.urdf"

        if robot_type is ROBOT_TYPE.STICK_RIGHT_TOP:
            urdf_robot_path = urdf_root + "panda_stick_right_top.urdf"
        elif robot_type is ROBOT_TYPE.STICK_RIGHT_FRONT:
            urdf_robot_path = urdf_root + "panda_stick_right_front.urdf"
        elif robot_type is ROBOT_TYPE.STICK_RIGHT_BACK:
            urdf_robot_path = urdf_root + "panda_stick_right_back.urdf"
        elif robot_type is ROBOT_TYPE.STICK_LEFT_TOP:
            urdf_robot_path = urdf_root + "panda_stick_left_top.urdf"
        elif robot_type is ROBOT_TYPE.STICK_LEFT_FRONT:
            urdf_robot_path = urdf_root + "panda_stick_left_front.urdf"
        elif robot_type is ROBOT_TYPE.STICK_LEFT_BACK:
            urdf_robot_path = urdf_root + "panda_stick_left_back.urdf"

        if robot_type is ROBOT_TYPE.STICK_RIGHT_TOP or robot_type is ROBOT_TYPE.STICK_RIGHT_FRONT or robot_type is ROBOT_TYPE.STICK_RIGHT_BACK:
            self.robot_stick_index = 13
        elif robot_type is ROBOT_TYPE.STICK_LEFT_TOP or robot_type is ROBOT_TYPE.STICK_LEFT_FRONT or robot_type is ROBOT_TYPE.STICK_LEFT_BACK:
            self.robot_stick_index = 12

        return urdf_robot_path
