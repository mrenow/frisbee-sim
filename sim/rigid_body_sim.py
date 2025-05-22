import time
from vpython import *
import numpy as np
from sim.rigid_body import RigidBodySimulation
from sim.math import Quaternion



class RigidBodyShape:
    
    InertiaTensor = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ])
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
            obj.pos = R (initial_vectors['pos'])
            obj.axis = R (initial_vectors['axis'])
            obj.up = R (initial_vectors['up'])

    

class DudeBro(RigidBodyShape):
    InertiaTensor =  np.diag([2.0, 1.0, 0.5])
    def __init__(self):
        """
        Stick figure dude
        """


        self.head = sphere(radius=1, pos=vector(0, 1, 0), color=color.red)
        self.body = ellipsoid(pos=vector(0, 0, 0), axis=vector(1, 0, 0), length=1, height=2, width=1, color=color.blue)
        self.left_arm = box(pos=vector(-1, 0, 0), axis=vector(1, 0, 0), length=2, height=0.5, width=0.5, color=color.green)
        self.right_arm = box(pos=vector(1, 0, 0), axis=vector(-1, 0, 0), length=2, height=0.5, width=0.5, color=color.green)
        self.left_leg = box(pos=vector(-0.5, -2, 0), axis=vector(0, 1, 0), length=2, height=0.5, width=0.5, color=color.green)
        self.right_leg = box(pos=vector(0.5, -2, 0), axis=vector(0, 1, 0), length=2, height=0.5, width=0.5, color=color.green)
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
            {'pos': vector(obj.pos), 'axis': vector(obj.axis), 'up': vector(obj.up)}
            for obj in self.objects
        ]

class Frisbee(RigidBodyShape):
    InertiaTensor = np.diag([0.1, 0.5, 0.1])
    def __init__(self):
        """
        Frisbee shape
        """
        radius = 1
        depth = 0.1
        self.body = cylinder(pos=vector(0, -depth/2, 0), axis=vector(0, depth, 0), radius=radius, color=color.blue)
        self.xaxis = box(pos=vector(0, 0, 0), axis=vector(1, 0, 0), length=2*radius, height=2*depth, width=depth, color=color.red)
        self.zaxis = box(pos=vector(0, 0, 0), axis=vector(0, 0, 1), length=2*radius, height=2*depth, width=depth, color=color.green)

        self.objects = [
            self.body,
            self.xaxis,
            self.zaxis
        ]
        self.orignal_params = [
            {'pos': vector(obj.pos), 'axis': vector(obj.axis), 'up': vector(obj.up)}
            for obj in self.objects
        ]



class RigidBodyVPythonSimulation:
    def __init__(self, obj, q0, omega0):
        """
        Initialize the VPython visualization for the rigid body simulation.

        Parameters:
        - I: Moments of inertia (3x3 diagonal matrix)
        - q0: Initial orientation quaternion (4x1 array)
        - omega0: Initial angular velocity in body frame (3x1 array)
        - dt: Time step for the simulation
        """
        self.object = obj
        I = obj.InertiaTensor
        self.simulation = RigidBodySimulation(I, q0, omega0)


        # State quantities
        

        self.body_torque = np.array([0.0, 0.0, 0.0])

        self.inertial_torque = np.array([0.0, 0.0, 0.0])
        # Computed quantities
        self.angular_momentum_intertial = Quaternion.rotate(q0, I @ omega0)
        self.angular_velocity_intertial = Quaternion.rotate(q0, omega0)
        self.kinetic_energy = 0.5 * omega0 @ I @ omega0


        # initialize arrows and other objexcts
        
        self.arrow_scale = 2.0
        
        self.angular_velocity_arrow = arrow(pos=vector(0, 0, 0), axis=self.arrow_scale*vector(*self.angular_velocity_intertial), shaftwidth=0.1, color=color.yellow)
        self.angular_momentum_arrow = arrow(pos=vector(0, 0, 0), axis=self.arrow_scale*vector(*self.angular_momentum_intertial), shaftwidth=0.1, color=color.blue)

        self.body_torque_arrow = arrow(pos=vector(0, 0, 0), axis=self.arrow_scale*vector(*Quaternion.rotate(q0, self.body_torque)), shaftwidth=0.1, color=color.green)


        self.inertial_torque_arrow = arrow(pos=vector(0, 0, 0), axis=self.arrow_scale*vector(*self.inertial_torque), shaftwidth=0.1, color=color.red)

        self.total_torque = Quaternion.rotate(q0, self.body_torque) + self.inertial_torque

        self.floor = box(pos=vector(0, -5, 0), axis=vector(1, 0, 0), length=100, height=0.1, width=100, color=color.white)
        self.axis_x = arrow(pos=vector(0, -5, 0), axis=vector(100, 0, 0), shaftwidth=0.1, color=color.red)
        self.axis_z = arrow(pos=vector(0, -5, 0), axis=vector(0, 0, 100), shaftwidth=0.1, color=color.green)


        # Text overlay

        self.top_left_text = label(pos=vector(-10, 20, -10), text="Angular Velocity", height=20, border=4)



        # Controls
        self.mouse_focus = False
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
            self.inertial_torque = mag* np.array([0.0, 0.0, 1.0])
        elif "j" in keysdown():
            self.inertial_torque = mag* np.array([0.0, 0.0, -1.0])
        elif "i" in keysdown():
            self.inertial_torque = mag* np.array([0.0, 1.0, 0.0])
        elif "k" in keysdown():
            self.inertial_torque = mag* np.array([0.0, -1.0, 0.0])
        else:
            self.inertial_torque = mag* np.array([0.0, 0.0, 0.0])
        # self.inertial_torque = np.array([0.0, 0.0, 1.0])


        self.total_torque = Quaternion.rotate(self.simulation.q, self.body_torque) + self.inertial_torque

    def update_quantities(self):
        q = self.simulation.q
        self.angular_momentum_intertial = Quaternion.rotate(q, self.simulation.I @ self.simulation.omega)
        self.angular_velocity_intertial = Quaternion.rotate(q, self.simulation.omega)
        self.kinetic_energy = 0.5 * self.simulation.omega @ self.simulation.I @ self.simulation.omega
    
        
    
    def update_graphics(self):
        # Get the current quaternion and angular velocity
        q = self.simulation.q

        # Convert quaternion to rotation matrix and apply to the body
        R = self.quaternion_to_vector_rotation(q)
        self.object.update(R)


        self.angular_velocity_arrow.axis=self.arrow_scale*vector(*self.angular_velocity_intertial)
        self.angular_momentum_arrow.axis=self.arrow_scale*vector(*self.angular_momentum_intertial)


        self.body_torque_arrow.axis=self.arrow_scale*vector(*Quaternion.rotate(q, self.body_torque))
        self.inertial_torque_arrow.axis=self.arrow_scale*vector(*self.inertial_torque)
        self.body_torque_arrow.pos = self.angular_momentum_arrow.pos + self.angular_momentum_arrow.axis
        self.inertial_torque_arrow.pos = self.angular_momentum_arrow.pos + self.angular_momentum_arrow.axis

        text_pieces = [
            f"Angular Momentum: {self.angular_momentum_intertial[0]:.2f}, {self.angular_momentum_intertial[1]:.2f}, {self.angular_momentum_intertial[2]:.2f}\n" \
            f"Angular Velocity: {self.angular_velocity_intertial[0]:.2f}, {self.angular_velocity_intertial[1]:.2f}, {self.angular_velocity_intertial[2]:.2f}\n" \
            f"Torque: {self.total_torque[0]:.2f}, {self.total_torque[1]:.2f}, {self.total_torque[2]:.2f}\n" \
            f"Kinetic Energy: {self.kinetic_energy:.2f}\n" \
            f"Camera Position: {scene.camera.pos.x:.2f}, {scene.camera.pos.y:.2f}, {scene.camera.pos.z:.2f}\n" \
            f"Camera Up: {scene.camera.up.x:.2f}, {scene.camera.up.y:.2f}, {scene.camera.up.z:.2f}\n" \
            f"Camera Axis: {scene.camera.axis.x:.2f}, {scene.camera.axis.y:.2f}, {scene.camera.axis.z:.2f}\n" \
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
        if ' ' in keylist: # move up
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
        while t_end < 0 or t < t_end:
            rate(200)  # Limit the simulation to 100 frames per second

            # Step the simulation
            dt = time.time() - t
            self.update_controls(t, dt)
            self.simulation.update(dt, self.total_torque)
            self.update_quantities()
            self.update_camera(dt)
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
            
            scene.camera.rotate(angle=dx*0.01, axis=vector(0, 1, 0), origin=scene.camera.pos)
            scene.camera.rotate(angle=dy*0.01, axis=vector(1, 0, 0), origin=scene.camera.pos)
        
if __name__ == "__main__":
    # Example parameters
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    omega0 = np.array([0.0, 5.0, 1])

    sim = RigidBodyVPythonSimulation(Frisbee(), q0, omega0)

    sim.run(-1)