import time
from vpython import *
from .header import *
import numpy as np
from sim.rigid_body import FrisbeeSimulation
from sim.math import Quaternion


class RigidBodyShape:

    InertiaTensor = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])

    objects: list[standardAttributes]
    orignal_params: list[dict[str, vector]]

    def __init__(self):
        ...

    def update(self, R):
        """
        Update the position and orientation of the rigid body shape using their original positions.

        Parameters:
        - R: Rotation matrix (3x3 array)
        """
        for obj, initial_vectors in zip(self.objects, self.orignal_params):
            # Update the position using the rotation matrix
            obj.pos = R(initial_vectors['pos'])
            obj.axis = R(initial_vectors['axis'])
            obj.up = R(initial_vectors['up'])


class DudeBro(RigidBodyShape):
    InertiaTensor = np.diag([2.0, 1.0, 0.5])

    def __init__(self):
        """
        Stick figure dude
        """

        self.head = sphere(radius=1, pos=vector(0, 1, 0), color=color.red)
        self.body = ellipsoid(pos=vector(0, 0, 0), axis=vector(
            1, 0, 0), length=1, height=2, width=1, color=color.blue)
        self.left_arm = box(pos=vector(-1, 0, 0), axis=vector(1, 0, 0),
                            length=2, height=0.5, width=0.5, color=color.green)
        self.right_arm = box(pos=vector(1, 0, 0), axis=vector(-1, 0, 0),
                             length=2, height=0.5, width=0.5, color=color.green)
        self.left_leg = box(pos=vector(-0.5, -2, 0), axis=vector(0, 1, 0),
                            length=2, height=0.5, width=0.5, color=color.green)
        self.right_leg = box(pos=vector(0.5, -2, 0), axis=vector(0, 1, 0),
                             length=2, height=0.5, width=0.5, color=color.green)
        self.left_leg.rotate(angle=-pi/6, axis=vector(0, 0, 1))
        self.right_leg.rotate(angle=pi/6, axis=vector(0, 0, 1))

        self.objects = [
            self.head,
            self.body,
            self.left_arm,
            self.right_arm,
            self.left_leg,
            self.right_leg
        ]
        self.orignal_params = [
            {'pos': vector(obj.pos), 'axis': vector(
                obj.axis), 'up': vector(obj.up)}
            for obj in self.objects
        ]


class Frisbee(RigidBodyShape):

    '''
    Represents the shape of a frisbee
    '''

    def __init__(self, mass=2, radius=1.0):
        """
        Frisbee shape
        """
        # Inertia of a thin disk
        self.InertiaTensor = 0.5 * radius * \
            radius * mass * np.diag([0.5, 1, 0.5])

        depth = 0.1
        self.body = cylinder(pos=vector(
            0, -depth/2, 0), axis=vector(0, depth, 0), radius=radius, color=color.blue)
        self.xaxis = box(pos=vector(0, 0, 0), axis=vector(
            1, 0, 0), length=2*radius, height=2*depth, width=depth, color=color.red)
        self.zaxis = box(pos=vector(0, 0, 0), axis=vector(
            0, 0, 1), length=2*radius, height=2*depth, width=depth, color=color.green)

        self.objects = [
            self.body,
            self.xaxis,
            self.zaxis
        ]
        self.orignal_params = [
            {'pos': vector(obj.pos), 'axis': vector(
                obj.axis), 'up': vector(obj.up)}
            for obj in self.objects
        ]


class FormulaInterface[T](Protocol):
    def __call__(self, simulation: T, deps: dict[str, f64array], /) -> f64array:
        ...


class Quantity[T]:
    '''
    A strategy of obtaining a quantity from the simulation
    '''

    def __init__(self, simulation: T, name: str, formula: FormulaInterface[T], depends: Sequence['Quantity'] = []):
        self.name = name
        self._simulation = simulation
        self._formula = formula
        self.value = formula(simulation, {d.name: d.value for d in depends})
        self.depends: Set[Quantity] = set(depends)
        self._needs_update = False

    def pre_update(self):
        if self._needs_update:
            return
        self._needs_update = True
        for dep in self.depends:
            dep._needs_update = True

    def update(self):
        if self._needs_update:
            self._needs_update = False
            for dep in self.depends:
                dep.update()
            # Update the value using the formula
            self.value = self._formula(
                self._simulation, {d.name: d.value for d in self.depends})

    def __str__(self):
        return f"{self.name}: {self.value}"


class DirectQuantity(Quantity[Any]):
    '''
    A simulation vector quantity that is not dependent on a simulation or other quantities
    this can be used as a constant value or an external variable.
    '''

    def __init__(self, name: str, initial_value: f64array):
        self.name = name
        self.value = initial_value
        self.depends = set()
        self._needs_update = False

    def set(self, value: f64array):
        self.value = value

    def update(self):
        pass


class ArrowQuantity[T](Quantity[T]):
    '''
    A simulation vector quantity that is represented by an arrow
    It's position may be derived via another quantity
    '''

    def __init__(self,
                 simulation: T,
                 name: str,
                 formula: FormulaInterface[T],
                 depends: Sequence[Quantity[Any]] = [],
                 origin: Quantity[Any] | None = None,
                 scale: float = 1.0,
                 shaftwidth: float = 0.1,
                 color: vector = color.white,
                 ):
        super().__init__(simulation, name, formula, depends)

        self.origin = origin if origin is not None else DirectQuantity(
            "origin", np.array([0, 0, 0]))

        self.scale = scale
        pos = vector(*self.origin.value)
        self.arrow = arrow(pos=pos, axis=scale*vector(*self.value),
                           shaftwidth=shaftwidth, color=color)

    def update(self):
        if self._needs_update:
            self._needs_update = False
            for dep in self.depends:
                dep.update()
            self.origin.update()
            # Update the value using the formula
            self.value = self._formula(
                self._simulation, {d.name: d.value for d in self.depends})
            self.arrow.axis = self.scale * vector(*self.value)
            if isinstance(self.origin, ArrowQuantity):
                pos = self.origin.arrow.axis + self.origin.arrow.pos
            else:
                pos = vector(
                    *(self.origin.value if self.origin else [0, 0, 0]))
            self.arrow.pos = pos


class RigidBodyVPythonSimulation:

    def __getitem__(self, name: str) -> f64array:
        """
        Get the value of a quantity by its name.

        Parameters:
        - name: Name of the quantity

        Returns:
        - value: Value of the quantity
        """
        if name in self.quantities:
            return self.quantities[name].value
        else:
            raise ValueError(f"Quantity '{name}' not found.")

    def add_quantity[T](self, quantity: Quantity[T]) -> Quantity[T]:
        """
        Add a quantity to the simulation.

        Parameters:
        - quantity: An instance of the Quantity class
        """
        if quantity.name in self.quantities:
            raise ValueError(
                f"Quantity with name {quantity.name} already exists.")
        self.quantities[quantity.name] = quantity
        return quantity

    def reset(self):
        """
        Reset the simulation to its initial state.
        """
        omega0 = np.linalg.inv(self.object.InertiaTensor) @ self.L0
        self.sim.set_state(self.object.InertiaTensor, self.q0, omega0, self.v0)

    def __init__(self, obj, q0, L):
        """
        Initialize the VPython visualization for the rigid body simulation.

        Parameters:
        - I: Moments of inertia (3x3 diagonal matrix)
        - q0: Initial orientation quaternion (4x1 array)
        - omega0: Initial angular velocity in body frame (3x1 array)
        - dt: Time step for the simulation
        """

        self.q0 = np.copy(q0)
        self.L0 = np.copy(L)
        self.v0 = np.array([1.0, 0.0, 0.0])  # Initial linear velocity

        self.object = obj
        I = obj.InertiaTensor
        self.sim = FrisbeeSimulation(I, q0, self.L0, np.array([1.0, 0.0, 0.0]))
        self.quantities: dict[str, Quantity] = {}

        # State quantities

        self.input_torque = DirectQuantity(
            "input_torque", np.array([0.0, 0.0, 0.0]))
        self.add_quantity(self.input_torque)
        # Computed quantities


        self.arrow_scale = 2.0


        av_q = self.add_quantity(
            ArrowQuantity(self.sim, "angular_velocity", lambda sim, deps: Quaternion.rot(sim.inertial.q, sim.body.w),
                          color=color.yellow,
                          scale=self.arrow_scale))

        am_q = self.add_quantity(
            ArrowQuantity(self.sim, "angular_momentum", lambda sim, deps: Quaternion.rot(sim.inertial.q, sim.body.I @ sim.body.w),
                          color=color.blue,
                          scale=self.arrow_scale))
        ke_q = self.add_quantity(
            Quantity(self.sim, "kinetic_energy", lambda sim,
                     deps: 0.5 * sim.body.w @ sim.body.I @ sim.body.w))


        orientation_arrow = self.add_quantity(
            ArrowQuantity(self.sim, "orientation", lambda sim, deps: Quaternion.rot(sim.inertial.q, np.array([0.0, 1.0, 0.0])),
                          color=color.white,
                          scale=self.arrow_scale,
                          shaftwidth=0.01))

        total_torque_arrow = self.add_quantity(
            ArrowQuantity(self.sim, "total_torque", lambda sim, deps: sim.inertial.T_net,
                          origin=am_q,
                          color=color.red,
                          scale=self.arrow_scale,
                          shaftwidth=0.04))

        velocity_arrow = self.add_quantity(
            ArrowQuantity(self.sim, "velocity", lambda sim, deps: Quaternion.rot(sim.inertial.q, sim.body.v),
                          color=color.cyan,
                          scale=self.arrow_scale,
                          shaftwidth=0.1))

        # Reference environment

        self.floor = box(pos=vector(0, -5, 0), axis=vector(1, 0, 0),
                         length=100, height=0.1, width=100, color=color.white)
        self.axis_x = arrow(pos=vector(
            0, -5, 0), axis=vector(100, 0, 0), shaftwidth=0.1, color=color.red)
        self.axis_z = arrow(pos=vector(
            0, -5, 0), axis=vector(0, 0, 100), shaftwidth=0.1, color=color.green)

        # Text overlay

        self.top_left_text = label(
            pos=vector(-10, 20, -10), text="Angular Velocity", height=20, border=4)

        # Controls
        self.mouse_focus = False
        self.running = True
        self.keymap = {}

        # Set movable camera

        scene.camera.pos = vector(0, 0, 5)
        scene.camera.axis = vector(0, 0, -5)
        scene.width = 1800
        scene.height = 900
        scene.autoscale = False
        scene.background = vector(0.5, 0.5, 0.5)
        scene.range = 5
        scene.center = vector(0, 0, 0)

        scene.bind('keydown', self.handle_input)
        scene.bind('mousemove', self.handle_mouse)

    def quaternion_to_vector_rotation(self, q):
        """
        Convert quaternion to a vpython vector rotation function
        """
        q = q / np.linalg.norm(q)
        # Get rotation axis and angle

        angle = 2 * np.arccos(q[0])
        s = np.sqrt(1 - q[0]**2)
        if s < 1e-6:
            axis = np.array([1, 0, 0])
        else:
            axis = q[1:] / s
        return lambda v: rotate(v, axis=vector(axis[0], axis[1], axis[2]), angle=angle)

    def update_controls(self, t, dt):
        """
        Update the control inputs for the simulation.

        Parameters:
        - t: Current time
        - dt: Time step
        """
        # Define a function to set the torque based on time in a repeating fashion
        t = t % 2.0  # Repeat every 2 seconds

        mag = 4

        if "l" in keysdown():
            self.input_torque.set(mag * np.array([0.0, 0.0, 1.0]))
        elif "j" in keysdown():
            self.input_torque.set(mag * np.array([0.0, 0.0, -1.0]))
        elif "i" in keysdown():
            self.input_torque.set(mag * np.array([0.0, 1.0, 0.0]))
        elif "k" in keysdown():
            self.input_torque.set(mag * np.array([0.0, -1.0, 0.0]))
        else:
            self.input_torque.set(mag * np.array([0.0, 0.0, 0.0]))

    def update_quantities(self):

        for quantity in self.quantities.values():
            quantity.pre_update()
        for quantity in self.quantities.values():
            quantity.update()

    def update_graphics(self):
        # Get the current quaternion and angular velocity
        q = self.sim.inertial.q

        # Convert quaternion to rotation matrix and apply to the body
        R = self.quaternion_to_vector_rotation(q)
        self.object.update(R)

        text_pieces = [
            f"Angular Momentum: {self['angular_momentum'][0]:.2f}, {self['angular_momentum'][1]:.2f}, {self['angular_momentum'][2]:.2f}\n"
            f"Angular Velocity: {self['angular_velocity'][0]:.2f}, {self['angular_velocity'][1]:.2f}, {self['angular_velocity'][2]:.2f}\n"
            f"Torque: {self['input_torque'][0]:.2f}, {self['input_torque'][1]:.2f}, {self['input_torque'][2]:.2f}\n"
            f"Kinetic Energy: {self['kinetic_energy']:.2f}\n"
            f"Camera Position: {scene.camera.pos.x:.2f}, {scene.camera.pos.y:.2f}, {scene.camera.pos.z:.2f}\n"
            f"Camera Up: {scene.camera.up.x:.2f}, {scene.camera.up.y:.2f}, {scene.camera.up.z:.2f}\n"
            f"Camera Axis: {scene.camera.axis.x:.2f}, {scene.camera.axis.y:.2f}, {scene.camera.axis.z:.2f}\n"
        ]

        self.top_left_text.text = ''.join(text_pieces)

    def update_camera(self, dt):
        """
        Update the camera position and orientation based on user input.
        """
        speed = 8
        keylist = keysdown()
        if 'a' in keylist:
            vec = cross(scene.camera.axis, scene.camera.up)
            vec = vec / mag(vec)
            scene.camera.pos += -speed*dt * vec
        if 'd' in keylist:
            vec = cross(scene.camera.axis, scene.camera.up)
            vec = vec / mag(vec)
            scene.camera.pos += speed*dt * vec
        if 's' in keylist:
            vec = scene.camera.axis - proj(scene.camera.axis, scene.camera.up)
            vec = vec / mag(vec)
            scene.camera.pos += -speed*dt * vec
        if 'w' in keylist:
            vec = scene.camera.axis - proj(scene.camera.axis, scene.camera.up)
            vec = vec / mag(vec)
            scene.camera.pos += speed*dt * vec
        if ' ' in keylist:  # move up
            scene.camera.pos += vector(0, speed*dt, 0)
        if 'shift' in keylist:
            scene.camera.pos += vector(0, -speed*dt, 0)

    def run(self, t_end):
        """
        Run the VPython simulation.

        Parameters:
        - t_end: End time for the simulation
        """
        t = time.time()
        simtime = 0
        while t_end < 0 or t < t_end:
            rate(200)  # Limit the simulation to 100 frames per second

            dt = time.time() - t
            self.update_controls(t, dt)
            self.update_camera(dt)
            if self.running:
                self.sim.set_control(torque=self['input_torque'])
                # Step the simulation
                self.sim.evolve(dt)
                self.update_quantities()
                simtime += dt

            self.update_graphics()

            t = time.time()

    def handle_input(self, event):
        """
        Handle keyboard input for controlling the simulation.

        Parameters:
        - event: Keyboard event
        """
        if event.key == 'esc':
            self.mouse_focus = not self.mouse_focus
        if event.key == 'p':
            self.running = not self.running

        if event.key == 'r':
            self.reset()

    def handle_mouse(self, event):
        """
        Mouse movement will be captured and used to rotate the camera 

        Parameters:
        - event: Mouse event
        """

        if self.mouse_focus:
            print(dir(event))
            dx = event.pos.x - scene.mouse.pos.x
            dy = event.pos.y - scene.mouse.pos.y

            scene.camera.rotate(
                angle=dx*0.01, axis=vector(0, 1, 0), origin=scene.camera.pos)
            scene.camera.rotate(
                angle=dy*0.01, axis=vector(1, 0, 0), origin=scene.camera.pos)


if __name__ == "__main__":
    # Example parameters
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    L_inertial = np.array([10.0, 3.0, 0])  # Initial angular momentum

    sim = RigidBodyVPythonSimulation(Frisbee(), q0, L_inertial)

    sim.run(-1)
