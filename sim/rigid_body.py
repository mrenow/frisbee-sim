from dataclasses import dataclass, field
from typing import Callable
import numpy as np

from sim.math import Quaternion
from .header import *
from functools import cached_property


@dataclass
class FrisbeeState:
    """
    A class representing the state of the frisbee simulation.
    This class holds the current orientation quaternion and angular velocity.
    """
    q: np.ndarray = field(default_factory=lambda: np.array(
        [1.0, 0.0, 0.0, 0.0], dtype=f64))  # Default to identity quaternion
    # Angular momentum in inertial frame, a conserved quantity
    L: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=f64))
    v: np.ndarray = field(default_factory=lambda: np.zeros(
        3, dtype=f64))  # Velocity in inertial frame

    def __add__(self, other: Self) -> Self:
        """
        Add two FrisbeeState objects together.
        """
        return self.__class__(
            q=self.q + other.q,
            L=self.L + other.L,
            v=self.v + other.v
        )

    def __sub__(self, other: Self) -> Self:
        """
        Subtract two FrisbeeState objects.
        """
        return self.__class__(
            q=self.q - other.q,
            L=self.L - other.L,
            v=self.v - other.v
        )

    def __mul__(self, scalar: float) -> Self:
        """
        Multiply the state by a scalar.
        """
        return self.__class__(
            q=self.q * scalar,
            L=self.L * scalar,
            v=self.v * scalar
        )

    def __iadd__(self, other: Self) -> Self:
        """
        In-place addition of two FrisbeeState objects.
        """
        self.q += other.q
        self.L += other.L
        self.v += other.v
        return self

    def __isub__(self, other: Self) -> Self:
        """
        In-place subtraction of two FrisbeeState objects.
        """
        self.q -= other.q
        self.L -= other.L
        self.v -= other.v
        return self

    def __imul__(self, scalar: float) -> Self:
        """
        In-place multiplication of the state by a scalar.
        """
        self.q *= scalar
        self.L *= scalar
        self.v *= scalar
        return self

    __rmul__ = __mul__
    __radd__ = __add__


@dataclass
class FrisbeeStaticValues:
    """
    A class representing any simulation values that are not determined by the evolution equation.
    These can represent constants or externally driven variables.
    """

    # Default to a diagonal matrix with small values
    I: f64array = field(default_factory=lambda: np.diag([0.01, 0.01, 0.01]))
    '''
    Moment of inertia of the frisbee in the body frame.
    '''
    g: float = 9.81

    # Driven variables
    T_ext: f64array = field(default_factory=lambda: np.zeros(3, dtype=f64))
    F_ext: f64array = field(default_factory=lambda: np.zeros(3, dtype=f64))

    @cached_property
    def I_inv(self) -> f64array:
        """
        Inverse of the moment of inertia matrix.
        This is computed as the inverse of the diagonal matrix I.
        """
        return np.linalg.inv(self.I)  # type: ignore


@dataclass
class FrisbeeComputationCache:
    """
    A class to hold computed values that can be reused in the simulation.
    This is useful for caching intermediate results to avoid redundant calculations.
    """
    derivative: FrisbeeState = field(default_factory=FrisbeeState)
    T_flutter: f64array = field(default_factory=lambda: np.zeros(3, dtype=f64))
    T_net: f64array = field(default_factory=lambda: np.zeros(3, dtype=f64))
    w_inertial: f64array = field(
        default_factory=lambda: np.zeros(3, dtype=f64))


class FrisbeeSimulation:
    '''
    A physical simulation of a frisbee.
    This class can be thought of as the core physics engine for the frisbee simulation
    all physical quantities should be derivable from the state of this class.
    '''

    def __init__(self, I: f64array, q0: f64array, L0: f64array, velocity0: f64array | None = None):
        """
        Initialize the rigid body simulation.

        Parameters:
        - I: Moments of inertia (3x3 diagonal matrix)
        - q0: Initial orientation quaternion (4x1 array)
        - L0: Initial angular velocity in body frame (3x1 array)
        - dt: Time step for the simulation
        """
        # Fixed quantities

        self.static: Final[FrisbeeStaticValues] = FrisbeeStaticValues()
        # Initial state
        self.state: Final[FrisbeeState] = FrisbeeState()
        # Computation cache
        self.cache: Final[FrisbeeComputationCache] = FrisbeeComputationCache()

        self.set_state(I, q0, L0, velocity0)

        # Frame views
        self.body = BodyFrameFrisbee(self.state, self.static, self.cache)
        self.inertial = InertialFrameFrisbee(
            self.state, self.static, self.cache)
        # self.aerodynamic = AerodynamicFrameFrisbee(self)

    def set_state(self, I: f64array, q0: f64array, L0: f64array, velocity0: f64array | None = None):
        """
        Set the initial state of the simulation.

        Parameters:
        - I: Moments of inertia (3x3 diagonal matrix)
        - q0: Initial orientation quaternion (4x1 array)
        - L0: Initial angular velocity in body frame (3x1 array)
        - velocity0: Initial velocity in inertial frame (3x1 array)
        """
        self.static.I = I
        self.state.q = q0 / np.linalg.norm(q0)
        self.state.L = L0
        self.state.v = velocity0 if velocity0 is not None else np.zeros(
            3, dtype=f64)

        # obtain initial cache values
        _ = self.state_derivative(self.state, cache=True)

    def set_control(self, torque=None, force=None):
        """
        Set the external inputs for the simulation.

        Parameters:
        - torque: External torque in inertial frame (3x1 array)
        - force: External force in inertial frame (3x1 array)
        """
        if torque is not None:
            self.static.T_ext = torque
        if force is not None:
            self.static.F_ext = force

    def state_derivative(self, state: FrisbeeState, cache: bool = False) -> FrisbeeState:

        L_body = Quaternion.rot_inv(state.q, state.L)
        

        
        w = self.static.I_inv @ L_body  # Angular velocity in body frame
        

        orientation = Quaternion.rot(state.q, np.array([0.0, 1.0, 0.0]))
        w_inertial = Quaternion.rot(state.q, w)
        T_flutter_coefficient = 0.1
        T_flutter = orientation * (w_inertial @ orientation) - w_inertial
        T_flutter = T_flutter * \
            np.linalg.norm(T_flutter) * T_flutter_coefficient

        T_net = self.static.T_ext + T_flutter

        deriv = FrisbeeState(
            L=T_net,
            q=Quaternion.angvel_to_deriv(state.q, w),
            v=0  # Assuming no acceleration for now
        )
        if cache:
            self.cache.derivative = deriv
            self.cache.T_flutter = T_flutter
            self.cache.w_inertial = w_inertial
            self.cache.T_net = T_net

        return deriv

    def normalize_state(self):
        """
        Normalize the current state to match constraints of the simulation.
        """
        self.state.q /= np.linalg.norm(self.state.q)

    @staticmethod
    def RK4_step(state: FrisbeeState, dt: float, deriv_func: Callable[Concatenate[FrisbeeState, ...], FrisbeeState]) -> None:
        """
        Perform a single RK4 step to evolve the simulation state in-place

        Parameters:
        - state: Current state of the simulationi
        - dt: Time step for the simulation
        - deriv_func: Function to compute the derivative of the state

        Returns:
        - new_state: New state after the RK4 step
        """
        k1 = deriv_func(state, cache=True)
        k2 = deriv_func(state + (dt / 2) * k1)
        k3 = deriv_func(state + (dt / 2) * k2)
        k4 = deriv_func(state + dt * k3)

        state += (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

    def evolve(self, dt):
        """
        Perform a single RK4 step to evolve the simulation.

        Parameters:
        - t: Current time
        """
        # # Calculate total torque and force
        # torque = self.constants.T_ext

        # Update the state in-place using RK4
        self.RK4_step(self.state, dt, self.state_derivative)

        # Normalize state
        self.normalize_state()


@dataclass
class BodyFrameFrisbee:
    '''
    A class representing the body frame of the frisbee.
    This is a fixed coordinate system that moves with the frisbee.
    '''
    state: FrisbeeState
    static: FrisbeeStaticValues
    cache: FrisbeeComputationCache

    @property
    def I(self):
        '''
        The moment of inertia of the frisbee in the body frame.
        This is a 3x3 diagonal matrix.
        '''
        return self.static.I

    @property
    def L(self):
        '''
        The angular momentum of the frisbee in the body frame.
        This is a 3x1 vector.
        '''
        return Quaternion.rot_inv(self.state.q, self.state.L)

    @property
    def w(self):
        '''
        The angular velocity of the frisbee in the body frame.
        This is a 3x1 vector.
        '''
        return self.static.I_inv @ self.L

    @property
    def w_dot(self):
        '''
        The angular acceleration of the frisbee in the body frame.
        This is a 3x1 vector.
        '''
        return self.static.I_inv @ (Quaternion.rot_inv(self.state.q, self.cache.T_net) - np.cross(self.w, self.L))

    @property
    def L_dot(self):
        '''
        The time derivative of the angular momentum of the frisbee in the body frame.
        This is a 3x1 vector.
        '''
        return Quaternion.rot_inv(self.state.q, self.cache.T_net) - np.cross(self.w, self.L)

    @property
    def v(self):
        '''
        The velocity of the frisbee in the body frame.
        This is a 3x1 vector.
        '''
        return Quaternion.rot_inv(self.state.q, self.state.v)

    @property
    def q(self):
        """
        Get the orientation quaternion of the frisbee in the body frame. This is always the vector [1, 0, 0, 0] since the body frame is aligned with the frisbee's orientation.
        """
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=f64)  # Identity quaternion for body frame

    @property
    def orientation(self) -> f64array:
        """
        Get the orientation quaternion of the frisbee in the body frame.

        Returns:
        - q: Orientation quaternion (4x1 array)
        """
        return np.array([0, 1, 0], dtype=f64)


@dataclass
class InertialFrameFrisbee:
    '''
    A class representing the inertial frame of the frisbee.
    This is a fixed coordinate system that does not move with the frisbee.
    '''
    state: FrisbeeState
    static: FrisbeeStaticValues
    cache: FrisbeeComputationCache

    @property
    def I(self) -> f64array:
        '''
        The moment of inertia of the frisbee in the inertial frame.
        This is a 3x3 diagonal matrix.
        '''
        return Quaternion.rot_billinear_form(self.q, self.static.I)

    @property
    def I_inv(self) -> f64array:
        '''
        The inverse of the moment of inertia of the frisbee in the inertial frame.
        This is a 3x3 diagonal matrix.
        '''
        return Quaternion.rot_billinear_form(self.q, self.static.I_inv)

    @property
    def L(self) -> f64array:
        '''
        The angular momentum of the frisbee in the inertial frame.
        This is a 3x1 vector.
        '''
        return self.state.L

    @property
    def w(self) -> f64array:
        '''
        The angular velocity of the frisbee in the inertial frame.
        This is a 3x1 vector.
        '''
        return self.I_inv @ self.L

    @property
    def w_dot(self) -> f64array:
        '''
        The angular acceleration of the frisbee in the inertial frame.
        This is a 3x1 vector.
        '''
        return Quaternion.rot(self.state.q, self.cache.derivative.L)

    @property
    def v(self) -> f64array:
        '''
        The velocity of the frisbee in the inertial frame.
        This is a 3x1 vector.
        '''
        return self.state.v

    @property
    def T_flutter(self) -> f64array:
        '''
        The flutter torque of the frisbee in the inertial frame.
        This is a 3x1 vector.
        '''
        return self.cache.T_flutter

    @property
    def q(self) -> f64array:
        """
        Get the orientation quaternion of the frisbee in the inertial frame.

        Returns:
        - q: Orientation quaternion (4x1 array)
        """
        return self.state.q

    @property
    def T_net(self):
        """
        Get the net torque acting on the frisbee in the inertial frame.

        Returns:
        - net_torque: Net torque (3x1 array)
        """
        return self.cache.T_net

    @property
    def orientation(self) -> f64array:
        """
        Get the orientation of the frisbee in the inertial frame.

        Returns:
        - orientation: Orientation vector (3x1 array)
        """
        return Quaternion.rot(self.state.q, np.array([0, 1, 0], dtype=f64))
    

# @dataclass
# class AerodynamicFrameFrisbee:
#     '''
#     A class representing the velocity frame of the frisbee.
#     This is a fixed coordinate system that moves with the frisbee.
#     '''
#     state: FrisbeeState
#     constants: FrisbeeConstants
#     cache: FrisbeeComputationCache


#     def frame_quaternion(self):
#         """
#         Get the aerodynamic frame of the frisbee based on its velocity.

#         Parameters:
#         - velocity: Velocity vector (3x1 array)

#         Returns:
#         - R: Rotation matrix (3x3 array)
#         """
#         q  = self.parent.q
#         velocity = self.parent.inertial.v
#         if np.linalg.norm(velocity) < 1e-6:
#             return q
#         # Get v1 rotated into the body frame
#         v1 = Quaternion.rotate(Quaternio_invnq), velocity)

#         cos_theta = v1[0]/(v1[0]**2 + v1[2]**2)**0.5
#         # Rotate q around the d1 axis to align with the velocity vector
#         return Quaternion.multiply(q, np.array([((1 + cos_theta)/2)**0.5, 0, ((1 - cos_theta)/2)**0.5, 0]))

#     @property
#     def v(self):
#         '''
#         The velocity of the frisbee in the inertial frame.
#         This is a 3x1 vector.
#         '''
#         return Quaternion.rotate(self.frame_quaternion(), self.parent.state.v)

#     @property
#     def w(self):
#         '''
#         The angular velocity of the frisbee in the inertial frame.
#         This is a 3x1 vector.
#         '''
#         return Quaternion.rotate(self.frame_quaternion(), self.parent._omega)

#     @property
#     def w_dot(self):
#         '''
#         The angular acceleration of the frisbee in the inertial frame.
#         This is a 3x1 vector.
#         '''
#         # Todo: this is not correct due to the fact that the frame is rotating
#         return Quaternion.rotate(self.frame_quaternion(), self.parent.body.w_dot)

#     @property
#     def L(self):
#         '''
#         The angular momentum of the frisbee in the inertial frame.
#         This is a 3x1 vector.
#         '''
#         return Quaternion.rotate(self.frame_quaternion(), self.parent.body.L)
