import numpy as np 
import sys
import os
import os
base_dir = os.getcwd()
from sys import platform
if platform != "linux" and platform != "linux2":
    dll_path = os.path.join(base_dir,"x64")
    os.add_dll_directory(dll_path)
# import starkit_ik_walk as sk
import  starkit_ik_walk as sk

from sys import stderr


phase = 0.0

dof_names = {
    "left_ankle_roll" : 0,
	"left_ankle_pitch" : 1,
	"left_bottom_knee" : 2,
	"left_upper_knee" : 3,
	"left_hip_pitch"  : 4,
	"left_hip_roll"  : 5,
	"left_hip_yaw"  : 6,
	"right_ankle_roll" : 7,
	"right_ankle_pitch" : 8,
	"right_bottom_knee" : 9,
	"right_upper_knee" : 10,
	"right_hip_pitch" : 11,
	"right_hip_roll" : 12,
	"right_hip_yaw" : 13,
    }

# labels = {}






class BalanceController:

    def __init__(self, physics, hip_joints, knee_joints, ankle_joints, kp_hip, kd_hip, kp_knee, kd_knee, kp_ankle, kd_ankle,target):

        self.params = sk.IKWalkParameters()

        self.params.enabledGain = 0.0
        self.params.stepGain = -0.01
        self.params.lateralGain = 0.0
        self.params.turnGain = 0.0
        self.params.freq = 1.4
        self.params.supportPhaseRatio = 0.0
        self.params.footYOffset = 0.01
        self.params.riseGain = 0.022
        self.params.swingGain = 0.02
        self.params.swingRollGain = 0.0
        self.params.swingPhase = 0.25
        self.params.swingPause = 0.0
        self.params.swingVel = 4.0
        self.params.trunkXOffset = 0.01
        self.params.trunkYOffset = 0.0
        self.params.trunkZOffset = 0.02
        self.params.trunkPitch = 0.19
        self.params.trunkRoll = 0.0
        # self.params.trunkPitchBackward = -0.02
        self.params.distHipToKnee = 0.098
        self.params.distKneeToAnkle = 0.098
        self.params.distAnkleToGround = 0.025
        self.params.distFeetLateral = 0.079
        self.params.extraLeftX = 0.0
        self.params.extraLeftY = 0.0
        self.params.extraLeftZ = 0.0
        self.params.extraRightX = 0.0
        self.params.extraRightY = 0.0
        self.params.extraRightZ = 0.0
        self.params.extraLeftYaw = 0.0
        self.params.extraLeftPitch = 0.0
        self.params.extraLeftRoll = 0.0
        self.params.extraRightYaw = 0.0
        self.params.extraRightPitch = 0.0
        self.params.extraRightRoll = 0.0


        
        self._physics = physics
        self._hip_joints = hip_joints
        self._knee_joints = knee_joints
        self._ankle_joints = ankle_joints
        self._kp_hip = kp_hip
        self._kd_hip = kd_hip
        self._kp_knee = kp_knee
        self._kd_knee = kd_knee
        self._kp_ankle = kp_ankle
        self._kd_ankle = kd_ankle

        self._kpz_hip = 0.01
        self._kdz_hip = 0.01
        self._kpz_knee = 0.02
        self._kdz_knee = 0.01
        self._kpz_ankle = 0.02
        self._kdz_ankle = 0.01

        self.target = target

        self.walkFlag=True
        self.xPos=0.019746263151372886

        # MuJoCo enum value for joints
        mjOBJ_JOINT = 3
        self._hip_dof_ids = self._physics.bind(hip_joints).element_id
        self._knee_dof_ids = self._physics.bind(knee_joints).element_id
        self._ankle_dof_ids = self._physics.bind(ankle_joints).element_id

        for hip_id in self._hip_dof_ids:
            self._physics.data.ctrl[hip_id] = 0.2
            # self._physics.data.qfrc_applied[hip_id] += hip_torque_z

        # Apply torques to knee joints
        for knee_id in self._knee_dof_ids:
            self._physics.data.ctrl[knee_id] = 0.2
            # self._physics.data.qfrc_applied[knee_id] += knee_torque_z

        for ankle_id in self._ankle_dof_ids:
            self._physics.data.ctrl[ankle_id] = 0.2
      

    def _get_dof_id(self, name, obj_type):
        joint_id = self._physics.model.name2id(name, obj_type)
        return self._physics.model.jnt_dofadr[joint_id]
    
    def send_command(self, command: sk.IKWalkOutputs):
            for name in dof_names:
                if "left_upper_knee" in name:
                  self._physics.data.ctrl[dof_names[name]] = 0.5 * getattr(command, 'left_knee')
                elif "right_upper_knee" in name:
                  self._physics.data.ctrl[dof_names[name]] = 0.5 * getattr(command, 'right_knee')
                elif "right_bottom_knee" in name:
                  self._physics.data.ctrl[dof_names[name]] = 0.5 * getattr(command, 'right_knee')
                elif "left_bottom_knee" in name:
                  self._physics.data.ctrl[dof_names[name]] = 0.5 * getattr(command, 'left_knee')
                else:
                  self._physics.data.ctrl[dof_names[name]] = getattr(command, name)
    def update_value(self, param_name, value):
        setattr(self.params, param_name, float(value))

    def run(self, target):
        global phase
        
        # Compute the center of mass (CoM) position and velocity
        com_pos = self._physics.data.subtree_com[0]
        com_pos[0] = com_pos[0] - self.xPos# Assuming 0 is the root body index
        com_pos[1] = com_pos[1] - 0.029123672185854297
        com_pos[2] = np.clip(-com_pos[2]+target[2], -0.3, 0.3)
        # print(com_pos)
        com_vel = self._physics.data.cvel[0][:3]  # Linear velocity of the CoM
        # Compute desired hip, knee, and ankle torques using PD control
        x_torque = - self._kp_hip * com_pos[0]   # PD control for hip
        y_torque = -self._kp_knee * com_pos[1] + self._kd_knee * com_vel[1]  # PD control for knee
        z_torque = -self._kp_ankle * com_pos[2] + self._kd_ankle * com_vel[2]  # PD control for ankle
        if target[0] - com_pos[0] >0.23:
            pass
                # if self.walkFlag:
                #     # self.xPos=com_pos[0]
                #     self.params.trunkPitch = 0.22
                #     self.params.trunkXOffset = 0.01
                #     self.params.trunkYOffset = 0.0
                #     self.params.trunkZOffset = 0.01
                #     self.params.enabledGain=1.0
                #     self.params.stepGain = 0.02
                #     self.params.turnGain = -0.0
                    

                # if target[0] - com_pos[0] < 0.14:
                #     self.walkFlag=False
                #     self.params.stepGain = 0.0
                #     self.params.enabledGain=0.0
                # elif target[0] - com_pos[0] >0.18:
                #     self.walkFlag=True


       
    
        else: 
            self.walkFlag=False
            # self.xPos=com_pos[0]+0.1
            self.approchFlag = False
            self.params.stepGain = 0.0
            self.params.turnGain = -0.0
            self.params.enabledGain=0.0
            

            # self.update_value("trunkXOffset",np.clip(self.params.trunkXOffset + x_torque,-0.0,0.03))
            # self.update_value("trunkYOffset",np.clip(self.params.trunkYOffset + y_torque,-0.03,0.03))
            # # self.update_value("trunkZOffset",np.clip(self.params.trunkZOffset + z_torque,0.01,0.04))
            # # self.update_value("trunkPitch",np.clip(self.params.trunkPitch + x_torque,-0.7,0.7))
            # self.update_value("trunkXOffset",np.clip(self.params.trunkXOffset + x_torque,-0.04,0.04))
       
        


            self.params.enabledGain=0.0
        # print(target[0] - com_pos[0],)

        # self.update_value("trunkZOffset",self.params.trunkZOffset + z_torque)

        # print(z_torque)
        # update_value()
        # update_value()
        outputs = sk.IKWalkOutputs()
        if sk.IKWalk.walk(self.params, 0.004, phase, outputs):
              self.send_command(outputs)
              # global phase
              phase = outputs.phase
        else:
            print(" Inverse Kinematics error. Position not reachable.", file=stderr)
        # trunkZOffset
        # for hip_id in self._hip_dof_ids:
        #     self._physics.data.ctrl[hip_id] = 0.2
        #     # self._physics.data.qfrc_applied[hip_id] += hip_torque_z

        # # Apply torques to knee joints
        # for knee_id in self._knee_dof_ids:
        #     self._physics.data.ctrl[knee_id] = -0.2
        #     # self._physics.data.qfrc_applied[knee_id] += knee_torque_z

        # for ankle_id in self._ankle_dof_ids:
            # self._physics.data.ctrl[ankle_id] = 0.2

        # hip_torque_z = self._kpz_hip * (com_pos[2]+ target[0]) + self._kdz_hip * com_vel[2]  # PD control for hip
        # knee_torque_z = -self._kpz_knee * com_pos[2] - self._kdz_knee * com_vel[2]  # PD control for knee
        ankle_torque_z = self._kpz_ankle * com_pos[2] + self._kdz_ankle * com_vel[2]  # PD control for ankle

        hip_torque_roll = self._kp_hip * com_pos[1] + self._kd_hip * com_vel[1]  # PD control for hip
        ankle_torque = 200*self._kp_ankle * com_pos[0] + self._kd_ankle * com_vel[0]  # PD control for ankle

      
        # Apply torques to hip joints
        # for hip_id in self._hip_dof_ids:
        #     self._physics.data.qfrc_applied[hip_id] += hip_torque
        #     # self._physics.data.qfrc_applied[hip_id] += hip_torque_z

        # # Apply torques to knee joints
        # for knee_id in self._knee_dof_ids:
        #     self._physics.data.qfrc_applied[knee_id] += knee_torque
        #     # self._physics.data.qfrc_applied[knee_id] += knee_torque_z

        # # # Apply torques to ankle joints
        # for ankle_id in self._ankle_dof_ids:
        #     self._physics.data.qfrc_applied[ankle_id] = ankle_torque
        return self.walkFlag

            # self._physics.data.qfrc_applied[ankle_id] += ankle_torque_z
# Example usage
# D