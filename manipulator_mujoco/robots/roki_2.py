import os
from manipulator_mujoco.robots.arm import Arm

ROKI2_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/robots/roki2/roki.xml',
)

LEFT_ARM_JOINTS = (
    'left_shoulder_roll_joint',
    'left_shoulder_pitch_joint',
    'left_elbow_pitch_joint',
    'left_elbow_yaw_joint',
    'left_arm_yaw_joint',
    # 'left_mimic_joint',
    
    # 'right_shoulder_roll_joint',
    # 'right_shoulder_pitch_joint',
    # 'right_elbow_pitch_joint',
    # 'right_elbow_yaw_joint',
    # 'right_arm_yaw_joint',

)
RIGHT_ARM_JOINTS = (
    'right_shoulder_roll_joint',
    'right_shoulder_pitch_joint',
    'right_elbow_pitch_joint',
    'right_elbow_yaw_joint',
    'right_arm_yaw_joint',
    
    # 'right_shoulder_roll_joint',
    # 'right_shoulder_pitch_joint',
    # 'right_elbow_pitch_joint',
    # 'right_elbow_yaw_joint',
    # 'right_arm_yaw_joint',

)

SERVO_MAP = {
    'right_shoulder_roll' : [1,3],
    'right_shoulder_pitch': [2,3],
    'right_elbow_pitch': [3,3],
    'right_elbow_yaw': [4,3],
    'right_arm_yaw': [5,3],
    'left_shoulder_roll': [1,4],
    'left_shoulder_pitch': [2,4],
    'left_elbow_pitch': [3,4],
    'left_elbow_yaw': [4,4],
    'left_arm_yaw': [5,4],
    
    # 'right_shoulder_roll_joint',
    # 'right_shoulder_pitch_joint',
    # 'right_elbow_pitch_joint',
    # 'right_elbow_yaw_joint',
    # 'right_arm_yaw_joint',

}

HIP_JOINTS = (
    # 'left_hip_yaw_joint',
    # 'left_hip_roll_joint',
    'left_hip_pitch_joint',
    # 'right_hip_yaw_joint',
    # 'right_hip_roll_joint',
    'right_hip_pitch_joint',

)

KNEE_JOINTS = (
     'left_upper_knee_joint',
    'left_bottom_knee_joint',
     'right_upper_knee_joint',
    'right_bottom_knee_joint',

)

ANKLE_JOINTS = (
   
    'left_ankle_pitch_joint',
    # 'left_ankle_roll_joint',
    
   
    'right_ankle_pitch_joint',
    # 'right_ankle_roll_joint',
   
)

_LEFT_EEF_SITE = 'left_eef_site'
_RIGHT_EEF_SITE = 'left_eef_site'


_ATTACHMENT_SITE = 'attachment_site'

class Roki2(Arm):
    def __init__(self, name: str = None):
        super().__init__(ROKI2_XML, _LEFT_EEF_SITE, _RIGHT_EEF_SITE,_ATTACHMENT_SITE, LEFT_ARM_JOINTS,RIGHT_ARM_JOINTS, HIP_JOINTS, KNEE_JOINTS, ANKLE_JOINTS, name, SERVO_MAP)