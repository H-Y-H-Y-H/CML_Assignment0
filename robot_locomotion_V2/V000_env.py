import time
import numpy as np
import pybullet as p
import pybullet_data as pd
import gym
from gym.spaces import Box
import logging
import os

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class V000_sm_Env(gym.Env):
    """Custom environment for V000 robot simulation using PyBullet."""
    def __init__(self, config):
        self.config = config
        self.initialize_variables()
        self.setup_action_and_observation_space()
        p.setAdditionalSearchPath(pd.getDataPath())

    def initialize_variables(self):
        """Initialize or reset the variables for the environment."""
        self.stateID = None
        self.robotid = None
        self.v_p = None
        self.q = None
        self.p = None
        self.last_p = None
        self.obs = np.zeros(18)
        self.mode = p.POSITION_CONTROL
        self.n_sim_steps = self.config['n_sim_steps']
        self.motor_action_space = np.pi / 3
        self.friction = self.config['friction']
        self.initial_moving_joints_angle = self.sin_move(0, self.config['para'])

    def setup_action_and_observation_space(self):
        """Define action and observation space according to environment settings."""
        self.action_space = Box(low=-np.ones(16, dtype=np.float32), high=np.ones(16, dtype=np.float32))
        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32)

    def sin_move(self, ti, para, sep=16):
        """Calculate joint angles using sinusoidal movements."""
        angles = np.zeros(12)
        for i in range(4):
            angles[3*i] = para[i*2] * np.sin(ti / sep * 2 * np.pi + para[2+i]) + (0.2 if i % 2 == 0 else 0.45)
            angles[3*i+1] = para[6] * np.sin(ti / sep * 2 * np.pi + para[2+i]) - (0.5 if i % 2 == 0 else 0.7)
            angles[3*i+2] = para[8] * np.sin(ti / sep * 2 * np.pi + para[2+i]) + (0.28 if i % 2 == 0 else 0.54)
        return angles

    def step(self, a):

        self.act(a)
        obs_step = self.get_obs()

        reward = 3 * obs_step[1] - abs(obs_step[5]) - 0.5 * abs(obs_step[0]) + 1
        Done = self.check()


        return obs_step, reward, Done, {}

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

    def robot_location(self):
        position, orientation = p.getBasePositionAndOrientation(self.robotid)
        orientation = p.getEulerFromQuaternion(orientation)

        return position, orientation


    def get_obs(self):
        """Retrieve and return the current observation of the environment."""
        self.last_p = self.p
        self.p, self.q = p.getBasePositionAndOrientation(self.robotid)
        self.q = p.getEulerFromQuaternion(self.q)
        self.p,self.q = np.asarray(self.p), np.asarray(self.q)
        self.v_p = self.p - self.last_p
        joint_info = [p.getJointState(self.robotid, i) for i in range(12)]
        joint_vals = np.array([joint[0] for joint in joint_info])
        self.obs = np.concatenate([self.v_p, self.q, joint_vals])
        return self.obs

    def act(self, sin_para):
        """Apply actions to the robot in the simulation."""
        for sub_step in range(self.config['sub_step_num']):
            a = self.sin_move(sub_step, sin_para)
            a = np.clip(a, -1, 1) * self.motor_action_space
            for j, pos_value in enumerate(a):
                p.setJointMotorControl2(self.robotid, j, controlMode=self.mode, targetPosition=pos_value,
                                        force=self.config['force'], maxVelocity=self.config['maxVelocity'])
            for _ in range(self.n_sim_steps):
                p.stepSimulation()
            time.sleep(self.config['sleep_time'])

    def reset(self):
        """Reset the simulation to the initial state."""
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        p.setAdditionalSearchPath(pd.getDataPath())
        planeId = p.loadURDF("plane.urdf")
        p.changeDynamics(planeId, -1, lateralFriction=self.friction)
        robotStartPos = [0, 0, 0.3]
        robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robotid = p.loadURDF(self.config['urdf_path'], robotStartPos, robotStartOrientation, flags=p.URDF_USE_SELF_COLLISION)
        for i in range(12):
            p.setJointMotorControl2(self.robotid, i, controlMode=self.mode, targetPosition=self.initial_moving_joints_angle[i],
                                    force=self.config['force'], maxVelocity=100)
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


def run_simulation(config):
    """Runs the robot simulation for a specified number of epochs."""
    p.connect(p.GUI)  # Connect to the PyBullet GUI for visualization
    # p.connect(p.DIRECT) # Use this to delete GUI for speeding up the program

    np.random.seed(config['random_seed'])

    # Load parameters for the gait generator
    gait_parameters = np.loadtxt(config['param_path'])
    os.makedirs(config['log_path'], exist_ok=True)  # Ensure log directory exists

    # Initialize the environment
    env = V000_sm_Env(config)
    env.sleep_time = config['sleep_time']
    _ = env.reset()


    SANS_data = []  # Sensor and Actuator data storage
    for epoch in range(config['N_epoch']):
        obs = env.resetBase()
        all_rewards = []
        for step in range(config['num_steps']):
            action = np.random.normal(gait_parameters, scale=config['gait_gaussian'])
            next_obs, reward, done, _ = env.step(action)
            SANS_data.append(np.hstack((obs, action, next_obs)))

            obs = next_obs
            all_rewards.append(reward)
            if done:
                break

        # Example logging
        logging.info(f"Epoch {epoch + 1}: Total Reward: {sum(all_rewards)}")

    p.disconnect()

if __name__ == '__main__':


    # Usage example
    config = {
        'para': np.loadtxt("CAD2URDF/para.csv"),
        'urdf_path': "CAD2URDF/V000/urdf/V000.urdf",
        'sleep_time': 1/960,
        'n_sim_steps': 30,
        'sub_step_num': 16,
        'force': 1.8,
        'maxVelocity': 1.5,
        'friction':0.99,
        'robot_name': 'V000',
        'param_path': "CAD2URDF/para.csv",
        'log_path': "data/babbling/",
        'random_seed': 2022,
        'gait_gaussian': 0.2,
        'num_steps': 6,
        'N_epoch': 100
    }

    run_simulation(config)