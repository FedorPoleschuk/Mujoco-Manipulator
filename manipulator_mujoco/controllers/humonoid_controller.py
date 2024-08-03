class HumanoidController:
    def __init__(self, L_arm_controller, R_arm_controller, balance_controller):
        self.l_arm_controller = L_arm_controller
        # self.r_arm_controller = R_arm_controller
        self._balance_controller = balance_controller

    def run(self, arm_target):
        walk = self._balance_controller.run(arm_target)
        # print
        # self.r_arm_controller.run(arm_target,walk=walk)
        self.l_arm_controller.run(arm_target,walk=walk)

# Example usage

