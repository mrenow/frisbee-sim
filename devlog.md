# May 22
Got rigidbody sim working fairly well.
Learned that rotational behaviour cane be rather chaotic, as it depends on:
1. Angular momentum, which is conserved, but changes it's orientation relative the body
3. Angular velocity, which is determined by the body orientation relative to angular momentum, and determines the future body orientation.

## On discs
Due to rotational symmetry, it seems that in all cases, the angular velocity vector rotates around the torque vector in a circle.
I don't currently have a good model for the speed of this rotation, but it can vary wildly. The angular velocity's rate of precession can be significantly faster than the angular
velocity itself.
- Appears to be proportional to angular momentum
It also appears to be the case that applying a torque aligned with AM does not cause the angle between AM and AV to change.

This is significant because it seems to suggest that 1. Frisbee wobble is easy to create and 2. Frisbee wobble does not self correct.
However in practice, we see this is not true. I suspect the answer lies in the decay of kinetic energy.


## Effect of kinetic energy
This can be derived from the intersection of the energy ellipse with the momentum sphere.
- System energy is at a local minimum when angular momentum vector and torque vector point in the same direction along the axis of maximum inertia
- System energy is at a local maximum when angular momentum and torque point in the same direction along the axis of minimum inertia. However, if the system
loses energy without a loss in angular momentum (idk, frictional forces not aligned with angular momentum maybe) this should cause the angular velocity and orientation
to process back to the low-energy configuration. Which is good news for frisbees.


## Funny stuff
Counterintuitively, it is possible to apply a torque that lengthens the angular momentum vector, yet decreases the total system kinetic energy. This happens when the angular
velocity vector is 

## Application to frisbees
Its probable that the fluttering of frisbees loses a lot of energy while negiglibly affecting angular momentum, since the torques produced by the fluttering effect would 
act perpendicularly to the AM.



## Future work
- Define a velocity vector (doesn't actually have to do anything for now)
- Define a new frame with respect to the velocity vector
- Start adding aerodynamic torques to the frisbee and see if they make any meaningful difference to the kinetic energy during flutter, if they produce the results we expect.


Investigate whats going on with the inertia tensor. You mean I can just do I^-1 L = Ω? that would be lossless probably

We can actually just analyse the stationary frisbee, and not actually move it with the velocity vector. The physics all stays the same.



## May 23
Added flutter torque. Appears to work well in most cases - gyroscopic stability & also rotational decay when the disc is rotating along it's intermediate axis.