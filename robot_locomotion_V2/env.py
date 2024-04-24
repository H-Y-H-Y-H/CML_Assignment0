import os
from torch.utils.data import Dataset, DataLoader
import time
import pybullet_data as pd
import pybullet as p
import gym
import numpy as np

class V000_sm_Env(gym.Env):
    def __init__(self, para, robot_camera=False, urdf_path='CAD2URDF'):

        self.stateID = None
        self.robotid = None
        # delta position
        self.v_p = None
        # orientation
        self.q = None
        # position
        self.p = None
        # initialize last position
        self.last_p = None
        # observation
        self.obs = [0] * 18
        # configurations for robot joints
        self.mode = p.POSITION_CONTROL
        self.maxVelocity = 1.5  # lx-224 0.20 sec/60degree = 5.236 rad/s
        self.force = 1.8
        self.sleep_time = 0  # decrease the value if it is too slow.

        self.joint_moving_idx = [0,1,2,3,4,5,6,7,8,9,10,11]
        self.n_sim_steps = 30
        self.motor_action_space = np.pi / 3
        self.urdf_path = urdf_path
        self.friction = 0.99
        self.robot_view_path = None
        # Each action contains 16 sub steps to achieve sin gait.
        self.sub_step_num = 16

        # CPG controller:
        self.initial_moving_joints_angle = self.sin_move(0,para)

        self.action_space = gym.spaces.Box(low=-np.ones(16, dtype=np.float32), high=np.ones(16, dtype=np.float32))
        self.observation_space = gym.spaces.Box(low=-np.ones(18, dtype=np.float32) * np.inf,
                                                high=np.ones(18, dtype=np.float32) * np.inf)

        self.log_obs = []
        self.log_action = []
        self.count = 0

        p.setAdditionalSearchPath(pd.getDataPath())

    def sin_move(self, ti, para, sep=16):
        # print(para)
        s_action = np.zeros(12)
        # print(ti)
        s_action[0] = para[0] * np.sin(ti / sep * 2 * np.pi + para[2]) + 0.2  # left   hind
        s_action[3] = para[1] * np.sin(ti / sep * 2 * np.pi + para[3]) + 0.45  # left   front
        s_action[6] = para[1] * np.sin(ti / sep * 2 * np.pi + para[4]) - 0.45  # right  front
        s_action[9] = para[0] * np.sin(ti / sep * 2 * np.pi + para[5]) - 0.2  # right  hind

        s_action[1] = para[6] * np.sin(ti / sep * 2 * np.pi + para[2]) - 0.5  # left   hind
        s_action[4] = para[7] * np.sin(ti / sep * 2 * np.pi + para[3]) - 0.7  # left   front
        s_action[7] = para[7] * np.sin(ti / sep * 2 * np.pi + para[4]) - 0.7  # right  front
        s_action[10] = para[6] * np.sin(ti / sep * 2 * np.pi + para[5]) - 0.5  # right  hind

        s_action[2] = para[8] * np.sin(ti / sep * 2 * np.pi + para[2]) + 0.28  # left   hind
        s_action[5] = para[9] * np.sin(ti / sep * 2 * np.pi + para[3]) + 0.54  # left   front
        s_action[8] = para[9] * np.sin(ti / sep * 2 * np.pi + para[4]) + 0.54  # right  front
        s_action[11] = para[8] * np.sin(ti / sep * 2 * np.pi + para[5]) + 0.28  # right  hind

        return s_action

    def get_obs(self):
        self.last_p = self.p
        # self.last_q = self.q
        self.p, self.q = p.getBasePositionAndOrientation(self.robotid)
        self.q = p.getEulerFromQuaternion(self.q)
        self.p, self.q = np.array(self.p), np.array(self.q)

        # Delta position and orientation
        self.v_p = self.p - self.last_p
        # self.v_q = self.q - self.last_q
        # if self.v_q[2] > 1.57:
        #     self.v_q[2] = self.q[2] - self.last_q[2] - 2 * np.pi
        # elif self.v_q[2] < -1.57:
        #     self.v_q[2] = (2 * np.pi + self.q[2]) - self.last_q[2]

        jointInfo = [p.getJointState(self.robotid, i) for i in self.joint_moving_idx]
        jointVals = np.array([[joint[0]] for joint in jointInfo]).flatten()
        self.obs = np.concatenate([self.v_p, self.q, jointVals]) # delta position, orientation and joint values.

        return self.obs

    def act(self, sin_para):
        for sub_step in range(self.sub_step_num):
            a = self.sin_move(sub_step, sin_para)
            a = np.clip(a, -1, 1)
            a *= self.motor_action_space

            for j in range(12):
                pos_value = a[j]
                p.setJointMotorControl2(self.robotid, self.joint_moving_idx[j], controlMode=self.mode,
                                        targetPosition=pos_value,
                                        force=self.force,
                                        maxVelocity=self.maxVelocity)

            for _ in range(self.n_sim_steps):
                p.stepSimulation()
            time.sleep(self.sleep_time)

    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(pd.getDataPath())
        planeId = p.loadURDF("plane.urdf")
        p.changeDynamics(planeId, -1, lateralFriction=self.friction)

        robotStartPos = [0, 0, 0.3]
        robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

        self.robotid = p.loadURDF(self.urdf_path, robotStartPos, robotStartOrientation, flags=p.URDF_USE_SELF_COLLISION,
                                  useFixedBase=0)

        p.changeDynamics(self.robotid, 2, lateralFriction=self.friction)
        p.changeDynamics(self.robotid, 5, lateralFriction=self.friction)
        p.changeDynamics(self.robotid, 8, lateralFriction=self.friction)
        p.changeDynamics(self.robotid, 11, lateralFriction=self.friction)

        for j in range(12):
            pos_value = self.initial_moving_joints_angle[j]
            p.setJointMotorControl2(self.robotid, self.joint_moving_idx[j], controlMode=self.mode,
                                    targetPosition=pos_value, force=self.force, maxVelocity=100)

        for _ in range(100):
            p.stepSimulation()
        self.stateID = p.saveState()
        self.p, self.q = p.getBasePositionAndOrientation(self.robotid)

        self.q = p.getEulerFromQuaternion(self.q)
        self.p, self.q = np.array(self.p), np.array(self.q)

        return self.get_obs()

    def resetBase(self):
        p.restoreState(self.stateID)

        self.last_p = 0
        self.p, self.q = p.getBasePositionAndOrientation(self.robotid)

        self.q = p.getEulerFromQuaternion(self.q)
        self.p, self.q = np.array(self.p), np.array(self.q)

        return self.get_obs()

    def step(self, a):

        self.act(a)
        obs_step = self.get_obs()

        reward = 3 * obs_step[1] - abs(obs_step[5]) - 0.5 * abs(obs_step[0]) + 1
        Done = self.check()

        self.count += 1

        return obs_step, reward, Done, {}

    def robot_location(self):
        position, orientation = p.getBasePositionAndOrientation(self.robotid)
        orientation = p.getEulerFromQuaternion(orientation)

        return position, orientation

    def check(self):
        pos, ori = self.robot_location()
        # if abs(pos[0]) > 0.5:
        #     abort_flag = True
        if abs(ori[0]) > np.pi / 6 or abs(ori[1]) > np.pi / 6: # or abs(ori[2]) > np.pi / 6:
            abort_flag = True
        # elif pos[1] < -0.04:
        #     abort_flag = True
        # elif abs(pos[0]) > 0.2:
        #     abort_flag = True
        else:
            abort_flag = False
        return abort_flag





if __name__ == '__main__':

    robot_name = 'V000'


    p.connect(p.GUI)
    # p.connect(p.DIRECT)
    np.random.seed(2022)
    # Load parameters for gait generator
    para = np.loadtxt("CAD2URDF/para.csv")
    gait_gaussian = 0.2
    # Gait parameters and sans data path:
    log_pth = "data/babbling/"
    os.makedirs(log_pth, exist_ok=True)


    # Initialize an environment
    env = V000_sm_Env(para, urdf_path="CAD2URDF/%s/urdf/%s.urdf" % (robot_name, robot_name))
    env.sleep_time = 1/960 # set 0 to speed up

    obs = env.reset()
    # Every epoisde the robot runs 6 steps.
    num_step = 6
    rcmd_a = para
    step_times = 0
    r_record = -np.inf
    SANS_data = []
    N_epoch = 100
    for i in range(N_epoch):
        all_rewards = []
        action_list = []
        r = 0
        obs = env.resetBase()
        action = np.random.normal(rcmd_a, scale=gait_gaussian)
        action_list.append(action)

        for i in range(num_step):
            step_times += 1
            next_obs, r, done, _ = env.step(action)
            sans = np.hstack((obs, action, next_obs))
            SANS_data.append(sans)
            obs = next_obs
            if done:
                break
        pos, ori = env.robot_location()
        all_rewards.append(r)

    p.disconnect()

