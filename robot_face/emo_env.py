import pybullet as p
import time
import pybullet_data

JOINT_LIMITS = {
    "base_neck1": [-1.3, 1.3, 0],
    "neck1_neck2": [-0.65, 0.65, 0],
    "neck2_head": [-0.5, 0.5, 0],
    "R_eye_yaw": [-1, 1, 0],
    "R_eye_pitch": [-0.8, 0.8, 0],
    "L_eye_yaw": [-1, 1, 0],
    "L_eye_pitch": [-0.8, 0.8, 0],
    "Ruppereyelid": [-1, 0.5, -0.3],
    "Luppereyelid": [-1, 0.5, -0.3],
    "Rlowereyelid": [-0.5, 0.5, -0.1],
    "Llowereyelid": [-0.5, 0.5, -0.1],
}


class FaceRobotEnv:
    def __init__(self,
                 urdf_path="face_sim/urdf/face_sim.urdf",
                 use_gui=True,
                 debug_tool=True,
                 time_step=1. / 240.):
        """
        Initialize the Face Robot Environment.

        :param urdf_path: Path to the URDF file of the robot.
        :param use_gui: Whether to use the PyBullet GUI.
        :param debug_tool: Whether to enable debug sliders for continuous joints.
        :param time_step: The simulation time step.
        """
        self.use_gui = use_gui
        self.debug_tool = debug_tool
        self.time_step = time_step
        self.physics_client = p.connect(p.GUI if self.use_gui else p.DIRECT)

        # Add default search path for plane.urdf, etc.
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load the environment
        self.plane_id = p.loadURDF("plane.urdf", [0, 0, 0])
        p.setGravity(0, 0, -9.81)

        # Load the robot
        start_pos = [0, 0, 0.3]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(urdf_path, start_pos, start_orn, useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot_id)


        # Set the GUI view window position and orientation
        if self.use_gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=0.3,  # Distance from the object
                cameraYaw=10,  # Rotate 10 degrees horizontally
                cameraPitch=-20,  # Look down at -20 degrees
                cameraTargetPosition=[0, 0, 0.5]  # Focus on the robot
            )

        # Initialize debug sliders for continuous joints
        self.joint_sliders = {}
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]

            if joint_type == p.JOINT_REVOLUTE and joint_name in JOINT_LIMITS:
                # Get the normalized initial position
                initial_real_value = JOINT_LIMITS[joint_name][2]
                initial_normalized = self.inverse_scale_action(joint_name, initial_real_value)

                # Create a debug slider with initial position set
                slider = p.addUserDebugParameter(joint_name, 0.0, 1.0, initial_normalized)
                self.joint_sliders[i] = slider

                # Set the initial joint position in the simulator
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=i,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=initial_real_value
                )
    def inverse_scale_action(self, joint_name, real_value):
        """
        Convert a real joint value to its normalized [0,1] representation.

        :param joint_name: Name of the joint.
        :param real_value: Real joint value.
        :return: Normalized value in [0,1] range.
        """
        min_val, max_val, _ = JOINT_LIMITS[joint_name]
        return (real_value - min_val) / (max_val - min_val)

    def scale_action(self, joint_name, normalized_value):
        """
        Scale an action from [0,1] range to its actual joint limits.
        """
        min_val, max_val, _ = JOINT_LIMITS[joint_name]
        return min_val + normalized_value * (max_val - min_val)

    def step(self, action=None, num_steps=10):
        """
        Perform a simulation step.

        :param action: Dictionary of joint_id to target position values in [0,1] range.
        :param num_steps: Number of simulation steps to run for each action.
        """
        if self.debug_tool:
            # Read slider values (normalized) and scale to real joint limits
            for joint_id, slider_id in self.joint_sliders.items():
                joint_info = p.getJointInfo(self.robot_id, joint_id)
                joint_name = joint_info[1].decode("utf-8")

                if joint_name in JOINT_LIMITS:
                    normalized_value = p.readUserDebugParameter(slider_id)
                    scaled_value = self.scale_action(joint_name, normalized_value)
                    p.setJointMotorControl2(
                        bodyUniqueId=self.robot_id,
                        jointIndex=joint_id,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=scaled_value
                    )
        else:
            # Apply external action commands if provided
            if action:
                for joint_id, normalized_value in action.items():
                    joint_info = p.getJointInfo(self.robot_id, joint_id)
                    joint_name = joint_info[1].decode("utf-8")

                    if joint_name in JOINT_LIMITS:
                        scaled_value = self.scale_action(joint_name, normalized_value)
                        p.setJointMotorControl2(
                            bodyUniqueId=self.robot_id,
                            jointIndex=joint_id,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=scaled_value
                        )

        # Run simulation for a fixed number of steps
        for _ in range(num_steps):
            p.stepSimulation()
            time.sleep(self.time_step)


    def disconnect(self):
        """ Disconnects from the PyBullet simulation. """
        p.disconnect()


if __name__ == "__main__":
    # Example usage
    env = FaceRobotEnv(use_gui=True, debug_tool=True)

    try:
        for _ in range(100000):  # Run simulation for 1000 steps
            env.step()
    except KeyboardInterrupt:
        print("Simulation interrupted by user.")

    # Disconnect from PyBullet
    env.disconnect()
