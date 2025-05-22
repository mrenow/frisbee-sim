import numpy as np

from sim.math import Quaternion


class RigidBodySimulation:
    def __init__(self, I, q0, omega0):
        """
        Initialize the rigid body simulation.

        Parameters:
        - I: Moments of inertia (3x3 diagonal matrix)
        - q0: Initial orientation quaternion (4x1 array)
        - omega0: Initial angular velocity in body frame (3x1 array)
        - dt: Time step for the simulation
        """
        self.I = I
        self.I_inv = np.linalg.inv(I)
        # State variables
        self.q = q0 / np.linalg.norm(q0)  # Normalize orientation quaternion
        self.omega = omega0  # angular velocity in body frame

    def external_torque(self, t):
        """
        Define external torque as a function of time.
        Override this method to specify custom torque.
        """
        return np.array([0.0, 0.0, 0.0])


    def update(self, dt, torque=None):
        """
        Perform a single RK4 step to evolve the simulation.

        Parameters:
        - t: Current time
        """
        torque = torque if torque is not None else np.empty(3)
        
        def omega_dot(omega):
            return self.I_inv @ (Quaternion.rotate(Quaternion.conjugate(self.q), torque) - np.cross(omega, self.I @ omega))

        def q_dot(q, omega):
            return Quaternion.angvel_to_deriv(q, omega)
        


        # RK4
        k1_omega = omega_dot(self.omega)
        k1_q = q_dot(self.q, self.omega)


        omega_2 = self.omega + 0.5 * dt * k1_omega
        k2_omega = omega_dot(omega_2)
        k2_q = q_dot(self.q + 0.5 * dt * k1_q, omega_2)
        
        omega_3 = self.omega + 0.5 * dt * k2_omega
        k3_omega = omega_dot(omega_3)
        k3_q = q_dot(self.q + 0.5 * dt * k2_q, omega_3)

        omega_4 = self.omega + dt * k3_omega
        k4_omega = omega_dot(omega_4)
        k4_q = q_dot(self.q + dt * k3_q, omega_4)
        
        self.omega += (dt / 6.0) * (k1_omega + 2 * k2_omega + 2 * k3_omega + k4_omega)
        self.q += (dt / 6.0) * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)
        self.q /= np.linalg.norm(self.q)  # Normalize quaternion


    # def simulate(self, t_end):
    #     """
    #     Run the simulation until a specified end time.

    #     Parameters:
    #     - t_end: End time for the simulation

    #     Returns:
    #     - results: List of (time, quaternion, angular velocity) tuples
    #     """
    #     t = 0.0
    #     results = []
    #     while t < t_end:
    #         results.append((t, self.q.copy(), self.omega.copy()))
    #         self.update(t)
    #         t += dt
    #     return results