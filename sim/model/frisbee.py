from dataclasses import dataclass, field
from typing import Callable
import numpy as np

from sim.model.math import Quaternion
from ..header import *
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
    x: np.ndarray = field(default_factory=lambda: np.zeros(
        3, dtype=f64))  # Position in inertial frame

    def __add__(self, other: Self) -> Self:
        """
        Add two FrisbeeState objects together.
        """
        return self.__class__(
            q=self.q + other.q,
            L=self.L + other.L,
            v=self.v + other.v,
            x=self.x + other.x
        )

    def __sub__(self, other: Self) -> Self:
        """
        Subtract two FrisbeeState objects.
        """
        return self.__class__(
            q=self.q - other.q,
            L=self.L - other.L,
            v=self.v - other.v,
            x=self.x - other.x
        )

    def __mul__(self, scalar: float) -> Self:
        """
        Multiply the state by a scalar.
        """
        return self.__class__(
            q=self.q * scalar,
            L=self.L * scalar,
            v=self.v * scalar,
            x=self.x * scalar
        )

    def __iadd__(self, other: Self) -> Self:
        """
        In-place addition of two FrisbeeState objects.
        """
        self.q += other.q
        self.L += other.L
        self.v += other.v
        self.x += other.x

        return self

    def __isub__(self, other: Self) -> Self:
        """
        In-place subtraction of two FrisbeeState objects.
        """
        self.q -= other.q
        self.L -= other.L
        self.v -= other.v
        self.x -= other.x
        return self

    def __imul__(self, scalar: float) -> Self:
        """
        In-place multiplication of the state by a scalar.
        """
        self.q *= scalar
        self.L *= scalar
        self.v *= scalar
        self.x *= scalar
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
    I: f64array = field(default_factory=lambda: np.diag(
        [0.00197/2, 0.00197, 0.00197/2]))
    '''
    Moment of inertia of the frisbee in the body frame.
    '''
    g: float = 9.81
    m: float = 0.175  # Mass of the frisbee in kg

    rho: float = 1.225
    area: float = 0.057
    radius: float = 0.269/2  # Radius of the frisbee in meters


    # Lift coefficients
    Cl0 = 0.15
    Clalpha: float = 1.4

    # Drag coefficients
    Cd0: float = 0.08
    Cdalpha: float = 2.72

    # Moment coefficients from lift and drag forces
    CM30: float = -0.01
    CM3alpha: float = 0.057
    CM3alpha_dot: float = 0.0025  # This is an angle of attack damping effect. I think this might be captured by the flutter already. Not sure.

    # Moment coefficients due to the rolling magnus moment
    CM1gamma: float = 0.006

    # Moments due to the deceleration of the disc
    CM2Nr: float = 0.000034

    # Coefficient of drag due to flutter
    C_flutter_plus: float = 1 # Flutter coefficient for the top side of the frisbee. This should be a bit lower than the drag coefficient for a flat plate.
    C_flutter_minus: float = 2.3 # Estimated as considerably higher than the linear drag coefficient for a flat plate perpendicular to flow.

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

    @cached_property
    def grav(self) -> f64array:
        """
        Gravitational force vector.
        This is computed as the mass times the gravitational acceleration in the negative z-direction.
        """
        return np.array([0.0, -self.g * self.m, 0.0], dtype=f64)

    @cached_property
    def AR(self) -> float:
        """
        Aspect ratio of the frisbee.
        This is computed as the radius divided by the area.
        """
        return 4/np.pi
    
    @cached_property
    def alpha0(self) -> float:
        """
        Angle of attack at which the lift coefficient is zero.
        """
        return -self.Cl0 / self.Clalpha 


    # The following properties just cache constant calculations for the simulation.
    @cached_property
    def C_flutter_torque(self) -> float:
        return self.C_flutter_plus + self.C_flutter_minus
    
    @cached_property
    def C_flutter_lift(self) -> float:
        return self.C_flutter_plus - self.C_flutter_minus

    @cached_property
    def r2(self) -> float:
        return self.radius ** 2
    
    @cached_property
    def pi_r4_rho_C_flutter_lift_on_8(self) -> float:
        return np.pi/8 * self.radius ** 4 * self.rho * self.C_flutter_lift
    @cached_property
    def r5_rho_C_flutter_torque_4_on_15(self) -> float:
        return 4/15 * self.radius ** 5 * self.rho * self.C_flutter_torque 
    
    @cached_property
    def rho_area(self) -> float:
        return self.rho * self.area
    
    @cached_property
    def rho_area_Cd_on_2(self) -> float:
        return self.rho_area * self.Cd0 /2
    
    @cached_property
    def rho_area_Cl_on_2(self) -> float:
        return self.rho_area * self.Cl0 /2
    
    @cached_property
    def rho_area_r(self) -> float:
        return self.rho_area * self.radius
    


@dataclass
class FrisbeeComputationCache:
    """
    A class to hold computed values that can be reused in the simulation.
    This is useful for caching intermediate results to avoid redundant calculations.
    """
    derivative: FrisbeeState = field(default_factory=FrisbeeState)
    T_flutter: f64array = field(default_factory=lambda: np.zeros(3, dtype=f64))
    T_net: f64array = field(default_factory=lambda: np.zeros(3, dtype=f64))
    F_net: f64array = field(default_factory=lambda: np.zeros(3, dtype=f64))
    w_inertial: f64array = field(
        default_factory=lambda: np.zeros(3, dtype=f64))
    flight_frame: f64array = field(
        # Default to identity quaternion
        default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=f64))
    lift_drag_inertial: f64array = field(
        default_factory=lambda: np.zeros(3, dtype=f64))  # Lift and drag forces in inertial frame
    


class FrisbeeSimulation:
    '''
    A physical simulation of a frisbee.
    This class can be thought of as the core physics engine for the frisbee simulation
    all physical quantities should be derivable from the state of this class.
    '''

    def __init__(self, q0: f64array, L0: f64array, v0: f64array | None = None):
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

        self.set_state(q0, L0, v0)

        # Frame views
        self.body = BodyFrameFrisbee(self.state, self.static, self.cache)
        self.inertial = InertialFrameFrisbee(
            self.state, self.static, self.cache)
        self.flight = FlightFrameFrisbee(self.state, self.static, self.cache)

    def set_state(self, q0: f64array, L0: f64array, v0: f64array | None = None, x0: f64array | None = None):
        """
        Set the initial state of the simulation.

        Parameters:
        - I: Moments of inertia (3x3 diagonal matrix)
        - q0: Initial orientation quaternion (4x1 array)
        - L0: Initial angular velocity in body frame (3x1 array)
        - v0: Initial velocity in inertial frame (3x1 array)
        """
        self.state.q = np.copy(q0) / np.linalg.norm(q0)
        self.state.L = np.copy(L0)
        self.state.v = np.copy(v0) if v0 is not None else np.zeros(3, dtype=f64)
        self.state.x = np.copy(x0) if x0 is not None else np.zeros(3, dtype=f64)

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

        flight_frame = FlightFrameFrisbee.to_inertial_quaternion(state)

        w_body = self.static.I_inv @ L_body  # Angular velocity in body frame

        # Flutter effect
        w_body_horisontal_sq = w_body[0]**2 + w_body[2]**2
        flutter_factor = np.array([w_body[0], 0, w_body[2]]) * w_body_horisontal_sq**0.5
        T_flutter = - flutter_factor * self.static.r5_rho_C_flutter_torque_4_on_15
        
        b1 = Quaternion.basis_vector(state.q, 1)
        F_flutter = - w_body_horisontal_sq * self.static.pi_r4_rho_C_flutter_lift_on_8 * b1 

        # FORCES
        v_body = Quaternion.rot_inv(state.q, state.v)
        v0, v1, v2 = v_body
        v_projected_length_sq = v0**2 + v2**2
        v_length_sq = v_projected_length_sq + v1**2

        alpha = -np.arctan(v1/v_projected_length_sq**0.5)

        Cl = self.static.Cl0
        Cl += self.static.Clalpha * alpha
        Cd = self.static.Cd0
        Cd += self.static.Cdalpha * (alpha - self.static.alpha0)**2


        # assert abs(v_sq - v_length_sq) <= 1e-4, f"Velocity squared mismatch: {v_sq} != {v_length_sq}. {np.linalg.norm(state.q)}"


        Arrhov2 = v_length_sq * self.static.rho_area_r

        # Lift and drag forces relative to the velocity vector (elevated by alpha, the angle of attack)
        lift_force_vframe = self.static.rho_area_Cl_on_2 * v_length_sq
        drag_force_vframe = self.static.rho_area_Cd_on_2 * v_length_sq

        # Transform lift and drag into flight frame
        lift_force_flight = lift_force_vframe * np.cos(alpha) + \
            drag_force_vframe * np.sin(alpha)

        drag_force_flight = lift_force_vframe * np.sin(alpha) - \
            drag_force_vframe * np.cos(alpha)
        lift_drag_inertial = Quaternion.rot(flight_frame, np.array(
            [drag_force_flight, lift_force_flight, 0], dtype=f64))
        
        F_net = np.copy(self.static.F_ext)  # External force in inertial frame
        F_net += self.static.grav
        F_net += lift_drag_inertial
        F_net += F_flutter
        a_net = F_net / self.static.m  # Net acceleration in inertial frame

        # TORQUES

        Cm3 = self.static.CM30 + self.static.CM3alpha * alpha

        # v_body_dot = Quaternion.rot_inv(
        #     state.q, a_net) - np.cross(w_body, v_body)
        # alpha_dot = (v_body_dot[0] * v0*v1
        #              + v_body_dot[2] * v2*v1
        #              - v_body_dot[1] * v_projected_length_sq
        #              ) / (v_length_sq * v_projected_length_sq**0.5)
        # Cm3 += self.static.CM3alpha_dot * alpha_dot
            

        m3 = Arrhov2 * Cm3

        # gamma_dot calculation
        # Angular velocity around the frisbee's axis
        rotation_speed = w_body[1]

        flight_to_body = FlightFrameFrisbee.to_body_quaternion(state)
        
        T_flutter = Quaternion.rot_inv(flight_to_body, T_flutter)


        m2 = -self.static.CM2Nr * rotation_speed

        T_net = np.copy(self.static.T_ext)

        flight_moments = np.zeros(3, dtype=f64)


        # Cm1 = (self.static.CM1gamma * rotation_speed + self.static.CM3alpha_dot *
        #        (w_body @ Quaternion.basis_vector(flight_to_body, 0)))
        # m1 = -Arrhov2 * Cm1
        # flight_moments += m1 * np.array([1, 0, 0])  # Magnus moment
        flight_moments += m2 * np.array([0, 1, 0])  # Spin
        flight_moments += m3 * np.array([0, 0, 1])  # Pitching moment

        flight_moments += T_flutter  # Flutter torque   
        T_net += Quaternion.rot(flight_frame, flight_moments)

        deriv = FrisbeeState(
            L=T_net,
            q=Quaternion.angvel_to_deriv(state.q, w_body),
            v=a_net,
            x=state.v
            # x=0,
        )
        if cache:
            print("flutter torque", T_flutter, "flutter force", F_flutter, "total force", F_net, "lift_drag_inertial", lift_drag_inertial)
            self.cache.derivative = deriv
            self.cache.T_flutter = T_flutter
            self.cache.w_inertial = Quaternion.rot(state.q, w_body)
            self.cache.T_net = T_net
            self.cache.F_net = F_net
            self.cache.flight_frame = flight_frame
            self.cache.lift_drag_inertial = lift_drag_inertial

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

    @property
    def flight_frame_e1(self) -> f64array:
        """
        Get the first basis vector of the flight frame of the frisbee.

        Returns:
        - flight_frame: flight frame quaternion (3x1 array)
        """
        return Quaternion.rot(self.cache.flight_frame, np.array([1, 0, 0], dtype=f64))
    
    @property
    def lift_drag_force(self) -> f64array:
        """
        Get the lift and drag forces acting on the frisbee in the inertial frame.

        Returns:
        - lift_drag: Lift and drag forces (3x1 array)
        """
        return self.cache.lift_drag_inertial
    


@dataclass
class FlightFrameFrisbee:
    '''
    A class representing the velocity frame of the frisbee.
    This is a fixed coordinate system that moves with the frisbee.
    '''
    state: FrisbeeState
    static: FrisbeeStaticValues
    cache: FrisbeeComputationCache

    @staticmethod
    def to_body_quaternion(state: FrisbeeState):
        """
        get the quaternion that transforms from the flight frame to the inertial frame.

        Parameters:
        - velocity: Velocity vector (3x1 array)

        Returns:
        - R: Rotation matrix (3x3 array)
        """
        q = state.q
        velocity = state.v
        if np.linalg.norm(velocity) < 1e-6:
            return q
        # Get v1 rotated into the body frame
        v_body = Quaternion.rot_inv(q, velocity)

        v = (v_body[0]**2 + v_body[2]**2)**0.5
        denominator = (2*v*(v + v_body[0]))**0.5

        # add rotation to q around the d1 axis to align with the velocity vector
        q_inertial_to_flight = np.array([(v + v_body[0])/denominator, 0, -v_body[2]/denominator, 0])
        # q_inertial_to_flight /= np.linalg.norm(q_inertial_to_flight)
        return q_inertial_to_flight

    @staticmethod
    def to_inertial_quaternion(state: FrisbeeState):
        """
        Get the quaternion that transforms vectors in the flight frame to the body frame of the frisbee.

        Parameters:
        - velocity: Velocity vector (3x1 array)

        Returns:
        - R: Rotation matrix (3x3 array)
        """
        q = Quaternion.mul(
            state.q, FlightFrameFrisbee.to_body_quaternion(state))
        # Normalize the quaternion to get rid of any numerical errors introduced by the multiplication
        q /= np.linalg.norm(q)
        return q

    @property
    def alpha(self) -> float:
        """
        Get the angle of attack of the frisbee in the flight frame.

        Returns:
        - alpha: Angle of attack (float)
        """
        v_flight = Quaternion.rot_inv(self.cache.flight_frame, self.state.v)
        # By definition, v_flight[2] is zero
        return np.arctan(v_flight[1]/v_flight[0])

    @property
    def alpha_grad_v(self) -> float:
        """
        Get the partial derivative of the angle of attack of the frisbee with respect to the velocity vector.

        Returns:
        - alpha_dot: Time derivative of the angle of attack (float)
        """

        # Body frame
        v_body = Quaternion.rot_inv(self.state.q, self.state.v)
        v0, v1, v2 = v_body
        projected_length_sq = v0**2 + v2**2
        length_sq = projected_length_sq + v1**2
        return np.array([-v0*v1, projected_length_sq, -v1*v2]) / (length_sq * projected_length_sq**0.5)

    # @property
    # def v(self):
    #     '''
    #     The velocity of the frisbee in the inertial frame.
    #     This is a 3x1 vector.
    #     '''
    #     return Quaternion.rot(self.cache.flight_frame, self.)

    # @property
    # def w(self):
    #     '''
    #     The angular velocity of the frisbee in the inertial frame.
    #     This is a 3x1 vector.
    #     '''
    #     return Quaternion.rot(self.cache.flight_frame, self)

    # @property
    # def w_dot(self):
    #     '''
    #     The angular acceleration of the frisbee in the inertial frame.
    #     This is a 3x1 vector.
    #     '''
    #     # Todo: this is not correct due to the fact that the frame is rotating
    #     return Quaternion.rot(self.cache.flight_frame, self)

    # @property
    # def L(self):
    #     '''
    #     The angular momentum of the frisbee in the inertial frame.
    #     This is a 3x1 vector.
    #     '''
    #     return Quaternion.rot(self.cache.flight_frame, self)
