import numpy as np

class JointEffortController:
    def __init__(
        self,
        physics,
        joints : None,
        ojoints : None,
        min_effort: np.ndarray,
        max_effort: np.ndarray,
        servo_map,
        simulation
    ) -> None:
        self._physics = physics
        self._joints = joints
        self._ojoints = ojoints
        self._min_effort = min_effort
        self._max_effort = max_effort
        self._simulation = simulation
        self._servo_map = servo_map

        # Create the joint to actuator map
        
        self._joint_to_actuator_map, self._ojoint_to_actuator_map, self._sim_joint_to_actuator_map, self._sim_ojoint_to_actuator_map = self._create_joint_to_actuator_map()
      
        

    def _create_joint_to_actuator_map(self):
            sim_joint_to_actuator_map = {}
            sim_ojoint_to_actuator_map = {}
            for joint in self._joints:
                try:
                    actuator_index = self._physics.model.name2id('roki/'+joint.name.split('_joint')[0], 'actuator')
                    sim_joint_to_actuator_map[joint.name] = actuator_index
                except KeyError:
                    print(f"Actuator for joint {joint.name} not found")
            for joint in self._ojoints:
                try:
                    actuator_index = self._physics.model.name2id('roki/'+joint.name.split('_joint')[0], 'actuator')
                    sim_ojoint_to_actuator_map[joint.name] = actuator_index
                except KeyError:
                    print(f"Actuator for joint {joint.name} not found")
            # return joint_to_actuator_map,ojoint_to_actuator_map
      
            joint_to_actuator_map = {}
            ojoint_to_actuator_map = {}
            for joint in self._servo_map:
                if "left" in joint:
                    joint_to_actuator_map[joint] = joint
                else:
                    ojoint_to_actuator_map[joint] = joint
            return joint_to_actuator_map,ojoint_to_actuator_map, sim_joint_to_actuator_map, sim_ojoint_to_actuator_map, 
            
                    
                
            
        

    def run(self, target) -> None:
        control = {}
        """
        Run the robot controller.

        Parameters:
            target (numpy.ndarray): The desired target efforts for the joints.
                                    The size of `target` should be (n_joints,) where n_joints is the number of joints.
        """
        # Clip the target efforts to ensure they are within the allowable effort range
        target_effort = np.clip(target, self._min_effort, self._max_effort)
      
        # Set the control signals for the specified actuators to the desired efforts
        for i, joint in enumerate(self._joints):
                if joint.name in self._sim_joint_to_actuator_map:
                    actuator_index = self._sim_joint_to_actuator_map[joint.name]
                    self._physics.data.ctrl[actuator_index] = target_effort[i]
        for i, joint in enumerate(self._ojoints):
                # print(JointEffortController)
                if 'shoulder' in joint.name or 'elbow_pitc' in joint.name:
                    if joint.name in self._ojoint_to_actuator_map:
                        actuator_index = self._sim_ojoint_to_actuator_map[joint.name]
                        self._physics.data.ctrl[actuator_index] = target_effort[i]
        # else: 
            # print( self._joint_to_actuator_map, self._joints)
        for i, joint in enumerate(self._joint_to_actuator_map):
                control[joint] = target_effort[i]
                    
        for i, joint in enumerate(self._ojoint_to_actuator_map):
            if 'shoulder' in joint or 'elbow_pitch' in joint:
                control[joint] = target_effort[i]
        print(control)
            # for joint in self._servo_map:
                
            
            
                 
            
            
        # return target_effort       
    def reset(self) -> None:
        pass

