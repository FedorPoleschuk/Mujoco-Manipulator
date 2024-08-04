import time
import numpy as np
from dm_control import mjcf
import mujoco
import mujoco.viewer
import gymnasium as gym
from gymnasium import spaces
from manipulator_mujoco.arenas import StandardArena
from manipulator_mujoco.robots import Roki2
from manipulator_mujoco.mocaps import Target
from manipulator_mujoco.controllers import IKArm, BalanceController, HumanoidController
import mujoco

class Roki2Env(gym.Env):

    metadata = {
        "render_modes": ["human", "rgb_array"],
        "render_fps": None,
    }  #

    def __init__(self, render_mode=None):
        # Observation space: state of the robot (positions, velocities, etc.)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64
        )

        # Action space: torques to be applied to the joints
        self.action_space = spaces.Box(
            low=-0.18, high=0.18, shape=(6,), dtype=np.float64
        )

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self._render_mode = render_mode

        ############################
        # Create MJCF model
        ############################
        
        # Checkerboard floor
        self._arena = StandardArena()

        # Mocap target that OSC will try to follow
        self._target = Target(self._arena.mjcf_model)

        # Robot
        self._robot = Roki2()

        # Attach robot to arena
        self._arena.attach_free(self._robot.mjcf_model)

        # Generate model
        self._physics = mjcf.Physics.from_mjcf_model(self._arena.mjcf_model)
        print(self._robot.servo_map)
        # Set up controllers
        self.left_arm_controller = IKArm(
            self._physics, 
            [], #self._robot.left_arm_joints, 
            [], #self._robot.right_arm_joints, 
            min_effort=np.array([-150.0] * len(self._robot.left_arm_joints)),
            max_effort=np.array([150.0] * len(self._robot.left_arm_joints)),
            servo_map = self._robot.servo_map
        )
        # self.right_arm_controller = IKArm(
        #     self._physics, 
        #     self._robot.right_arm_joints, 
        #     self._robot._arm_eef_site2,
        #     min_effort=np.array([-3.0] * len(self._robot.right_arm_joints)),
        #     max_effort=np.array([3.0] * len(self._robot.right_arm_joints)),
        # )

        self._balance_controller = BalanceController(
            self._physics, 
            self._robot._hip_joints, 
            self._robot._knee_joints, 
            self._robot._ankle_joints, 
            kp_hip=0.001,
            kd_hip=0.0000001,
            kp_knee=0.01,
            kd_knee=0.00001,
            kp_ankle=0.01,
            kd_ankle=0.00001,
            target=self._target.get_mocap_pose(self._physics),
        )
        
        self._humanoid_controller = HumanoidController(self.left_arm_controller,None, self._balance_controller)

        # For GUI and time keeping
        self._timestep = self._physics.model.opt.timestep
        self._viewer = None
        self._step_start = None

    def _get_obs(self) -> np.ndarray:
        # TODO: Come up with an observation that makes sense for your RL task
        return np.zeros(21)

    def _get_info(self) -> dict:
        # TODO: Come up with an info dict that makes sense for your RL task
        return {}

    def reset(self, seed=None, options=None) -> tuple:
        super().reset(seed=seed)

        # Reset physics
        with self._physics.reset_context():
            # Put arm in a reasonable starting position
            self._physics.bind(self._robot.left_arm_joints).qpos = np.array([0,0, 0,0 ,0])
            # self._physics.bind(self._robot.right_arm_joints).qpos = np.array([0, 0, -1, -1, 0])

            # Put target in a reasonable starting position
            self._target.set_mocap_pose(self._physics, position=[0.15, 0., 0.25], quaternion=[1, 0, 0, 0])

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def step(self, action: np.ndarray) -> tuple:
        # Use the action to control the arm
        # (assuming action is the target pose for the arm end-effector)

        # Get mocap target pose
        target_pose = self._target.get_mocap_pose(self._physics)
        # target_pose[4] = np.clip(target_pose[1].

        # Run humanoid controller to move to target pose
        self._humanoid_controller.run(target_pose)

        # Step physics
        self._physics.step()

        # Render frame
        if self._render_mode == "human":
            self._render_frame()

        # TODO: Come up with a reward, termination function that makes sense for your RL task
        observation = self._get_obs()
        reward = 0
        terminated = False
        info = self._get_info()

        return observation, reward, terminated, False, info

    def render(self) -> np.ndarray:
        """
        Renders the current frame and returns it as an RGB array if the render mode is set to "rgb_array".

        Returns:
            np.ndarray: RGB array of the current frame.
        """
        if self._render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self) -> np.ndarray:
        """
        Renders the current frame and updates the viewer if the render mode is set to "human".
        """
        if self._viewer is None and self._render_mode == "human":
            # Launch viewer
            self._viewer = mujoco.viewer.launch_passive(
                self._physics.model.ptr,
                self._physics.data.ptr,
            )
        if self._step_start is None and self._render_mode == "human":
            # Initialize step timer
            self._step_start = time.time()

        if self._render_mode == "human":
            # Render viewer
            self._viewer.sync()

            # TODO: Come up with a better frame rate keeping strategy
            time_until_next_step = self._timestep - (time.time() - self._step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

            self._step_start = time.time()

        else:  # rgb_array
            return self._physics.render()

    def close(self) -> None:
        """
        Closes the viewer if it's open.
        """
        if self._viewer is not None:
            self._viewer.close()
