import numpy as np
import pinocchio as pin
from typing import Tuple

# robot visualization
from pinocchio.visualize import GepettoVisualizer

import os
ropo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir))
# kinova_urdf = os.path.join(ropo_root, "gen3_7dof", "config", "GEN3_URDF_V12.urdf")
kinova_urdf = os.path.join(ropo_root, "gen3_7dof", "config", "kortex_description", "robots", "leonardo.urdf")


class SpringMassOscillationModel:
    def __init__(self):
        ## Mimic a spring-mass system with no damping
        # Robot model
        self.model = pin.buildModelFromUrdf(kinova_urdf)
        self.data = self.model.createData()
        # self.EE_frame_id = self.model.getFrameId("EndEffector") # for GEN3_URDF_V12.urdf
        self.EE_frame_id = self.model.getFrameId("tool_frame_joint") # for leonardo.urdf
        # changing the gravity vector
        self.model.gravity.linear = np.array([0, 9.80665, 0])

        # Oscillation parameters
        self.omega = 2 * np.pi  # Natural frequency
        self.k = 500.0  # Spring stiffness
        self.m = 1.0    # Apparent mass
        
        # Gravity constant
        self.g = 9.81  # m/s^2
        
        # Initial conditions
        self.z_0 = 0.5  # Initial z-position
        self.amplitude = 0.2  # Oscillation amplitude
        
    def compute_oscillation_trajectory(self, t: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute trajectory for gravity-compensated spring-mass oscillation
        
        Args:
            t: Current time
        
        Returns:
            x_d: Desired end-effector pose
            dx_d: Desired end-effector velocity 
            ddx_d: Desired end-effector acceleration
        """
        # Pure z-axis oscillation
        z = (
            self.z_0 + 
            self.amplitude * np.sin(self.omega * t)
        )
        
        # Velocity
        dz = self.amplitude * self.omega * np.cos(self.omega * t)
        
        # Acceleration (including gravity compensation)
        ddz = -self.amplitude * (self.omega**2) * np.sin(self.omega * t)
        
        # Construct full 6D vector (only z-axis moves)
        x_d = np.array([0, 0, z, 0, 0, 0])
        dx_d = np.array([0, 0, dz, 0, 0, 0])
        ddx_d = np.array([0, 0, ddz, 0, 0, 0])
        
        return x_d, dx_d, ddx_d
    
    def compute_impedance_control(self, q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
        """
        Compute impedance control torques for spring-mass oscillation
        
        Args:
            q: Joint positions
            dq: Joint velocities
            t: Current time
        
        Returns:
            tau: Joint torques
        """
        # Get Jacobian and dynamics
        J = self.get_jacobian(q)
        M, C, G = self.get_robot_dynamics(q, dq)
        
        # Compute desired trajectory
        x_d, dx_d, ddx_d = self.compute_oscillation_trajectory(t)
        
        # Current end-effector state
        x = self.get_end_effector_pose(q)
        dx = J @ dq
        
        # Compute errors
        e_x = x - x_d
        e_dx = dx - dx_d
        
        # Compute spring-like stiffness and damping (critically damped)
        K_p = self.k * np.eye(6)
        K_d = 2 * np.sqrt(self.k * self.m) * np.eye(6)
        
        # Compute acceleration command
        ddx_command = (
            ddx_d 
            - K_p @ e_x 
            - K_d @ e_dx
        )
        
        # Compute task-space inertia
        M_inv = np.linalg.inv(M)
        Lambda = np.linalg.inv(J @ M_inv @ J.T)
        
        # Compute control forces
        f_task = self.m * ddx_command
        f_dynamics = Lambda @ (J @ M_inv @ (C @ dq + G))
        
        # Compute joint torques
        tau = J.T @ (f_task + f_dynamics)
        
        return tau
    
    def get_robot_dynamics(self, q: np.ndarray, dq: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute robot dynamics"""
        M = pin.crba(self.model, self.data, q)
        C = pin.computeCoriolisMatrix(self.model, self.data, q, dq)
        pin.computeGeneralizedGravity(self.model, self.data, q)
        G = self.data.g
        return M, C, G
    
    def get_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Compute end-effector Jacobian"""
        pin.computeJointJacobians(self.model, self.data, q)
        return pin.getGeometryJacobian(self.model, self.data, self.model.njoints-1, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    
    def get_end_effector_pose(self, q: np.ndarray) -> np.ndarray:
        """Get current end-effector pose"""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.EE_frame_id)
        T = self.data.oMf[self.EE_frame_id]
        position = T.translation
        rotation = np.degrees(pin.rpy.matrixToRpy(T.rotation))
        return np.concatenate([position, rotation])

    def standard_to_pinocchio(self, q: np.ndarray) -> np.ndarray:
        """Convert standard joint angles (rad) to Pinocchio joint angles"""
        q_pin = np.zeros(self.model.nq)
        for i, j in enumerate(self.model.joints[1:]):
            if j.nq == 1:
                q_pin[j.idx_q] = q[j.idx_v]
            else:
                # cos(theta), sin(theta)
                q_pin[j.idx_q:j.idx_q+2] = np.array([np.cos(q[j.idx_v]), np.sin(q[j.idx_v])])
        return q_pin
    
    def pinocchio_to_standard(self, q_pin: np.ndarray) -> np.ndarray:
        """Convert Pinocchio joint angles to standard joint angles (rad)"""
        q = np.zeros(self.model.nv)
        for i, j in enumerate(self.model.joints[1:]):
            if j.nq == 1:
                q[j.idx_v] = q_pin[j.idx_q]
            else:
                q_back = np.arctan2(q_pin[j.idx_q+1], q_pin[j.idx_q])
                q[j.idx_v] = q_back + 2*np.pi if q_back < 0 else q_back
        return q

def main():
    pin_model = SpringMassOscillationModel()
    t = 0.0
    dt = 0.01
    
    # while t < 60.0:  # Run for 60 seconds
    #     # Get current robot state (simulated)
    #     q = get_current_joint_positions()
    #     dq = get_current_joint_velocities()
        
    #     # Compute control torques
    #     tau = controller.compute_impedance_control(q, dq, t)
        
    #     # Send torques to robot
    #     send_joint_torques(tau)
        
    #     t += dt

if __name__ == "__main__":
    main()