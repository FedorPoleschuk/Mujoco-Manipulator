import numpy as np

class JointEffortController:
    def __init__(
        self,
        physics,
        joints,
        min_effort: np.ndarray,
        max_effort: np.ndarray,
    ) -> None:
        self._physics = physics
        self._joints = joints
        self._min_effort = min_effort
        self._max_effort = max_effort

        # Create the joint to actuator map
        self._joint_to_actuator_map = self._create_joint_to_actuator_map()

    def _create_joint_to_actuator_map(self):
        joint_to_actuator_map = {}
        for joint in self._joints:
            try:
                actuator_index = self._physics.model.name2id('roki/'+joint.name.split('_joint')[0], 'actuator')
                joint_to_actuator_map[joint.name] = actuator_index
            except KeyError:
                print(f"Actuator for joint {joint.name} not found")
        return joint_to_actuator_map

    def run(self, target) -> None:
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
            if joint.name in self._joint_to_actuator_map:
                actuator_index = self._joint_to_actuator_map[joint.name]
                self._physics.data.ctrl[actuator_index] = target_effort[i]
        # return target_effort       
    def reset(self) -> None:
        pass

