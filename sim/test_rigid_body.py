import numpy as np
import pytest
from sim.rigid_body import RigidBodySimulation

def test_quaternion_normalization():
    I = np.diag([1.0, 1.0, 1.0])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    omega0 = np.array([0.0, 0.0, 0.0])
    dt = 0.01

    sim = RigidBodySimulation(I, q0, omega0, dt)
    assert np.isclose(np.linalg.norm(sim.q), 1.0), "Initial quaternion is not normalized"

def test_no_torque_no_rotation():
    I = np.diag([1.0, 1.0, 1.0])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    omega0 = np.array([0.0, 0.0, 0.0])
    dt = 0.01

    sim = RigidBodySimulation(I, q0, omega0, dt)
    results = sim.simulate(1.0)

    for t, q, omega in results:
        assert np.allclose(omega, np.array([0.0, 0.0, 0.0])), "Angular velocity should remain zero"
        assert np.allclose(q, q0), "Quaternion should remain constant"

def test_constant_torque():
    I = np.diag([1.0, 1.0, 1.0])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    omega0 = np.array([0.0, 0.0, 0.0])
    dt = 0.01

    class ConstantTorqueSimulation(RigidBodySimulation):
        def external_torque(self, t):
            return np.array([0.0, 0.0, 1.0])

    sim = ConstantTorqueSimulation(I, q0, omega0, dt)
    results = sim.simulate(1.0)

    for t, q, omega in results:
        assert omega[2] > 0, "Angular velocity should increase due to constant torque"

def test_quaternion_evolution():
    I = np.diag([1.0, 1.0, 1.0])
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    omega0 = np.array([0.0, 0.0, 1.0])
    dt = 0.01

    sim = RigidBodySimulation(I, q0, omega0, dt)
    results = sim.simulate(1.0)

    for t, q, omega in results:
        assert np.isclose(np.linalg.norm(q), 1.0), "Quaternion should remain normalized"

if __name__ == "__main__":
    pytest.main()