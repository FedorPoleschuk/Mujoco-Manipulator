import numpy as np
from scipy.optimize import minimize
import mujoco
import mujoco.viewer

# Load MuJoCo model
model_path = "roki.xml"
mujoco_model = mujoco.MjModel.from_xml_path(model_path)
mujoco_data = mujoco.MjData(mujoco_model)

# Viewer setup
viewer = mujoco.viewer.launch_passive(mujoco_model, mujoco_data)

# Initial configurations
q_initial = mujoco_data.qpos.copy()  # Adjust with actual initial joint positions if necessary
dq_initial = mujoco_data.qvel.copy()  # Adjust with actual initial joint velocities if necessary

# Define MPC parameters
horizon = 20
dt = 0.1
nu = mujoco_model.nu
nq = mujoco_model.nq
nv = mujoco_model.nv

# Cost function
def cost_func(x):
    q = x[:nq]
    dq = x[nq:nq+nv]
    u = x[nq+nv:]

    mujoco_data.qpos[:] = q
    mujoco_data.qvel[:] = dq
    mujoco_data.ctrl[:] = u
    mujoco.mj_forward(mujoco_model, mujoco_data)
    
    # Center of Mass (CoM) position
    com_pos = mujoco_data.subtree_com[0]  # Assuming the first body is the base
    com_des = np.array([0.0, 0.0, 0.23])  # Desired CoM position
    
    # Arm end-effector position
    arm_joint_id = mujoco_model.joint('left_gripper_joint').id  # Adjust with actual joint name
    arm_pos = mujoco_data.xpos[arm_joint_id]
    
    arm_des = np.array([0.5, 0.0, 1.0])  # Desired arm position
    
    # Compute the cost
    com_cost = np.sum((com_pos - com_des) ** 2)
    arm_cost = np.sum((arm_pos - arm_des) ** 2)
    control_cost = np.sum(u ** 2)
    
    return com_cost + arm_cost + control_cost

# Dynamics constraints
def dynamics_constraints(x):
    q = x[:nq]
    dq = x[nq:nq+nv]
    u = x[nq+nv:]

    mujoco_data.qpos[:] = q
    mujoco_data.qvel[:] = dq
    mujoco_data.ctrl[:] = u
    mujoco.mj_forward(mujoco_model, mujoco_data)
    
    ddq = mujoco_data.qacc[:].copy()
    
    # Ensure the constraints match the dimensions expected by the optimizer
    return np.concatenate((dq, ddq)) - np.concatenate((x[nq:nq+nv], np.zeros(nv)))

# Initial guess and bounds
x0 = np.concatenate((q_initial, dq_initial, np.zeros(nu)))
bounds = [(None, None)] * (nq + nv) + [(-np.inf, np.inf)] * nu

# Simulation loop
def apply_control(mujoco_data, u):
    mujoco_data.ctrl[:] = u
    mujoco.mj_step(mujoco_model, mujoco_data)

# Set initial state
q = q_initial
dq = dq_initial

for t in range(horizon):
    # Define optimization problem
    res = minimize(cost_func, x0, method='SLSQP', bounds=bounds, constraints={'type': 'eq', 'fun': dynamics_constraints})
    
    # Extract optimal control inputs
    optimal_u = res.x[nq+nv:]
    
    # Apply the control inputs
    apply_control(mujoco_data, optimal_u)
    
    # Update the state
    q = mujoco_data.qpos.copy()
    dq = mujoco_data.qvel.copy()
    
    # Update the initial guess for the next optimization
    x0 = np.concatenate((q, dq, optimal_u))
    
    viewer.sync()  # Sync the viewer with the updated data
