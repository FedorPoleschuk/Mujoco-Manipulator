from dm_control import mjcf
from manipulator_mujoco.utils.transform_utils import mat2quat
import numpy as np

class Arm():
    def __init__(self, xml_path, eef_site_name1, eef_site_name2, attachment_site_name, left_arm_joint_names=None, right_arm_joint_names=None, hip_joint_names=None, knee_joint_names=None, ankle_joint_names=None, name: str=None, servo_map=None):
        self._mjcf_root = mjcf.from_path(xml_path)
        if name:
            self._mjcf_root.model = name

        # Find MJCF elements that will be exposed as attributes.
        self.left_arm_joints = [self._mjcf_root.find('joint', name) for name in (left_arm_joint_names or [])]
        self.right_arm_joints = [self._mjcf_root.find('joint', name) for name in (right_arm_joint_names or [])]
        self._hip_joints = [self._mjcf_root.find('joint', name) for name in (hip_joint_names or [])]
        self._knee_joints = [self._mjcf_root.find('joint', name) for name in (knee_joint_names or [])]
        self._ankle_joints = [self._mjcf_root.find('joint', name) for name in (ankle_joint_names or [])]

        self._arm_eef_site1 = self._mjcf_root.find('site', eef_site_name1)
        self._arm_eef_site2 = self._mjcf_root.find('site', eef_site_name2)
        self._servo_map = servo_map

        self._attachment_site = self._mjcf_root.find('site', attachment_site_name)

    @property
    def mjcf_model(self):
        """Returns the `mjcf.RootElement` object corresponding to this robot."""
        return self._mjcf_root

    @property
    def hip_joints(self):
        """List of joint elements belonging to the hip."""
        return self._hip_joints

    @property
    def knee_joints(self):
        """List of joint elements belonging to the knee."""
        return self._knee_joints

    @property
    def ankle_joints(self):
        """List of joint elements belonging to the ankle."""
        return self._ankle_joints

    @property
    def eef_site1(self):
        """Wrist site of the arm (attachment point for the hand)."""
        return self._arm_eef_site1

    @property
    def eef_site2(self):
        """Wrist site of the arm (attachment point for the hand)."""
        return self._arm_eef_site2
    
    @property
    def servo_map(self):
        """Wrist site of the arm (attachment point for the hand)."""
        return self._servo_map

    def attach_tool(self, child, pos: list=[0, 0, 0], quat: list=[1, 0, 0, 0]) -> mjcf.Element:
        frame = self._attachment_site.attach(child)
        frame.pos = pos
        frame.quat = quat
        return frame

    def get_eef_pose1(self, physics):
        ee_pos = physics.bind(self._arm_eef_site1).xpos
        ee_quat = mat2quat(physics.bind(self._arm_eef_site1).xmat.reshape(3, 3))
        ee_pose = np.concatenate((ee_pos, ee_quat))
        return ee_pose

    def get_eef_pose2(self, physics):
        ee_pos = physics.bind(self._arm_eef_site2).xpos
        ee_quat = mat2quat(physics.bind(self._arm_eef_site2).xmat.reshape(3, 3))
        ee_pose = np.concatenate((ee_pos, ee_quat))
        return ee_pose
