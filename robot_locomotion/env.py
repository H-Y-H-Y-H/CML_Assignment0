import pybullet as p
import pybullet_data as pd
import numpy as np
import time


class Robo_env():
    def __init__(self):
        self.RoboID = None
        self.sleep_t = 1. / 240  # decrease the value if it is too slow.
        self.maxVelocity = 10
        self.force = 100
        self.n_sim_steps = 1
        self.startPos = [0, 0, 0.5]
        self.startOri = [np.pi / 2, 0, 0]
        self.initial_action = []
        self.jointIds = []
        self.reset()

    def step(self, action):
        for j in range(12):
            targetPos = float(action[j])
            print(targetPos)
            targetPos = self.jointDirections[j] * targetPos + self.jointOffsets[j]
            p.setJointMotorControl2(bodyIndex=self.RoboID,
                                    jointIndex=self.jointIds[j],
                                    targetPosition=targetPos,
                                    controlMode=p.POSITION_CONTROL,
                                    force=self.force,
                                    maxVelocity=self.maxVelocity)
        for _ in range(self.n_sim_steps):
            p.stepSimulation()
            time.sleep(1. / 500)

    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -10)
        planeId = p.loadURDF("plane.urdf")  # URDF Id = 0
        self.RoboID = p.loadURDF("laikago/laikago.urdf", self.startPos,
                                 p.getQuaternionFromEuler(self.startOri))  # URDF Id = 1

        self.jointOffsets = []
        self.jointDirections = [-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1]

        for i in range(4):
            self.jointOffsets.append(0)
            self.jointOffsets.append(-0.7)
            self.jointOffsets.append(0.7)


        for j in range(p.getNumJoints(self.RoboID)):
            p.changeDynamics(self.RoboID, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.RoboID, j)
            jointName = info[1]
            jointType = info[2]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                self.jointIds.append(j)


        with open(pd.getDataPath() + "/laikago/data1.txt", "r") as filestream:
            for line in filestream:
                currentline = line.split(",")
                joints = currentline[2:14]
                self.step(joints)
                self.initial_action = joints

if __name__ == '__main__':
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pd.getDataPath())  # optionally
    env1 = Robo_env()
    c = 0
    while 1:
        action_list = [-0.1] * 12
        env1.step(env1.initial_action)
        c += 1
