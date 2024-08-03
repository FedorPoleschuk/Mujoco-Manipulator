from manipulator_mujoco.controllers import JointEffortController
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

class OperationalSpaceController(JointEffortController):
    def __init__(

        self,
        physics,
        joints,
        eef_site,
        min_effort: np.ndarray,
        max_effort: np.ndarray,
        kp: float,
        ko: float,
        kv: float,
        vmax_xyz: float,
        vmax_abg: float,
        # walk: bool,
    ) -> None:
        L1 = rtb.RevoluteDH(d=0.0795, a=0., alpha=np.pi/2, qlim=[0,np.pi] ) #shoulder roll
        L2 = rtb.RevoluteDH(d=0.003,  alpha=np.pi/2, offset=np.pi/2+ 0.17453, qlim=[-np.pi,np.pi]) #elbow pitch
        L3 = rtb.RevoluteDH(d=0.08994, a=-0.005,alpha=np.pi/2, offset=0, qlim=[0,np.pi/2]) #elbow yaw
        L4 = rtb.RevoluteDH( alpha=np.pi/2, offset=np.pi/2, qlim=[-np.pi,np.pi])
        L5 = rtb.RevoluteDH( d=0.11, alpha=0, qlim=[-np.pi,np.pi])
        L6 = rtb.RevoluteDH( d=0, alpha=np.pi/2)
        self.robot = rtb.DHRobot([L1,L2,L3,L4,L5], name='3-DOF Robot')
        self.zeros = [0,0,np.pi/2,np.pi/2,0]
        
        
        super().__init__(physics, joints, min_effort, max_effort)

        self._eef_site = eef_site
        self._kp = kp
        self._ko = ko
        self._kv = kv
        self._vmax_xyz = vmax_xyz
        self._vmax_abg = vmax_abg

        self.walk=False

        self._joints = joints

        self._eef_id = self._physics.bind(eef_site).element_id
        self._jnt_dof_ids = self._physics.bind(joints).dofadr
        self.pos = [1.57,1.57, 0, 0, 0]
        self._dof = len(self._jnt_dof_ids)
        self.jntList={}
        self.id=0

        # Extract joint limits from the model
        # self._pos_limit, self._vel_limit = self._get_joint_limits()

        self._task_space_gains = np.array([self._kp] * 3 + [self._ko] * 3)
        self._lamb = self._task_space_gains / self._kv
        self._sat_gain_xyz = vmax_xyz / self._kp * self._kv
        self._sat_gain_abg = vmax_abg / self._ko * self._kv
        self._scale_xyz = vmax_xyz / self._kp * self._kv
        self._scale_abg = vmax_abg / self._ko * self._kv

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """Converts a quaternion into a rotation matrix."""
        # Normalize the quaternion
        norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        # Compute the rotation matrix elements
        R = np.array([
            [1 - 2 * (qy**2 + qz**2), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)],
            [2 * (qx*qy + qz*qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy*qz - qx*qw)],
            [2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx**2 + qy**2)]
        ])
        
        return R

    def create_transformation_matrix(self, x, y, z, qx, qy, qz, qw):
        """Creates a 4x4 transformation matrix from position and quaternion."""
        # Get the rotation matrix from the quaternion
        R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
        
        # Create the transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [-z, -x, y]
        # print(SE3(T))
        
        return SE3(T)


    def run(self, target, walk):
        # target pose is a 7D vector [x, y, z, qx, qy, qz, qw]
        target_pose = self.create_transformation_matrix(target[0]-0.019746263151372886,target[1] - 0.029123672185854297,target[2]-0.32,target[3],target[4],target[5],target[6])


        TQQ = self.robot.fkine([0,1.57,1.57,1.570,0])
        if self.id % 40 == 0:
            print(TQQ.t, "t=", target_pose.t)
        self.id +=1
        # Get the Jacobian matrix for the end-effector.
       
        # Get the joint velocities for the controlled DOF.
        
        # Initialize the task space control signal (desired end-effector motion).
        # u_task = np.zeros(6)
        p=np.zeros(5)

        p[0] = self.pos[1]
        p[1] = self.pos[0]
        p[2] = self.pos[2]
        p[3] = self.pos[3]
        p[4] = self.pos[4]
        

        # Calculate the task space control signal.
        # sol =  self.robot.ikine_LM(target_pose, q0=p+self.zeros, mask=[1,1,1,0,0,0], slimit=120, ilimit=2, joint_limits=True)
        
        

        # joint space control signal
        # u = np.zeros(self._dof)
        
        # # Add the task space control signal to the joint space control signal
        # u += np.dot(J.T, np.dot(Mx, u_task))

        # # Add damping to joint space control signal
        # u += -self._kv * np.dot(M, dq)

        # # Add gravity compensation to the target effort
        # u += self._physics.bind(self._joints).qfrc_bias

        # # Apply soft limiting to avoid joint limits
        # u = self._apply_soft_limits(u, dq)

        # Send the target effort to the joint effort controller
       
     

        if not self.walk:
            print('on')
            sol =  self.robot.ikine_LM(target_pose, q0=p+self.zeros, mask=[1,1,1,0,0,0], slimit=50, ilimit=3, joint_limits=True)
            
            print(sol)
            print('ommm')
            if sol.success:
                print('sdsds')
                u =sol.q-self.zeros
                sr = u[0]
                sp= u[1]
                ey = u[2]
                ep = u[3]
                ay = u[4]
                self.pos = [sp,sr,ep,ey,ay]
                super().run([sp,sr,ep,ey,ay])
                print('qqqqqqqqqqqqq',self.pos)

            else:
                self.walk=True
                # self.pos = [0,0, 1.57, 0, 0]
               


                super().run(np.zeros(5))
        else:
             if self.id % 150 == 0:
                    
                    sol =  self.robot.ikine_LM(target_pose, q0=p+self.zeros, mask=[1,1,1,0,0,0], slimit=180, ilimit=4, joint_limits=True)
                    print(sol)
                    if sol.success:
                        u =sol.q-self.zeros
                        sr = u[0]
                        sp= u[1]
                        ey = u[2]
                        ep = u[3]
                        ay = u[4]
                        self.pos = [sp,sr,ep,ey,ay]
                        print('off')
                        self.walk=False
