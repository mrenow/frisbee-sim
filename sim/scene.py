from vpython import *
#GlowScript 2.9 VPython

# David Brown
# This version completed in July 2020

# Right arrow = run forward
# Left arrow = run backward
# Down arrow = stop

# The rigid body consists of four particles with masses m1, m1, m2 and m2. 
# Each particle is a distance R from the origin. 
# The m1's are located at +x1vec and -x1vec.
# The m2's are located at +x2vec and -x2vec. 

# MOI = moment of inertia
# The smaller masses and principle axis with intermediate MOI are colored RED.
# The larger masses and principle axis with smallest MOI  are colored CYAN. 
# The principle axis with largest MOI is perpendicular to the plane of masses.
#
# The angular velocity is YELLOW
# The angular momentum is BLUE. 

##################################################
# ADJUSTABLE PARAMETERS 

# initial angular velocity:
omega = vector(0,0,1)

# masses and radius
totalmass = 10.0
m1overm2 = 0.1
m1 = m1overm2*totalmass/(1 + m1overm2)       
m2 = totalmass/(1 + m1overm2)
R = 1   

# Initial positions for masses:
# Start with m1's on z-axis and m2's on y-axis
# Rotate about the y-axis through angle psi
# Rotate about axis through m1 through angle chi
psi = 0.3*pi/180.0
chi = 0.0*(pi/180.0)
x1vec = vector(R*sin(psi),0,R*cos(psi))
x2vec = vector(-R*sin(chi)*cos(psi),R*cos(chi),R*sin(chi)*sin(psi)) 

# Show angular velocity
ShowAV = True
# Show angular momentum
ShowAM = True
# Show cyan trail for the smallest MOI axis
ShowCyan = False
# Show red trail for the intermediate MOI axis
ShowRed = False
ShowForcing = True
##################################################

# Set the camera angle
cameradistance = 3.1*R
angleup = 0.1*pi
angleside = 0.3*pi
scene.camera.pos = cameradistance*vector(cos(angleside)*cos(angleup),sin(angleside)*cos(angleup),sin(angleup))
scene.up = vector(0,0,1)
scene.camera.axis = -scene.camera.pos
scene.width=1920/2 + 1
scene.height=1080/2 + 1


forcing = 0.5*m1*vector(1,0,0)

# acceleration of mass m1. Apply an external force to m1 horisontally
def acc1(x1vec,v1vec,x2vec,v2vec):
    v1sqr = mag2(v1vec) 
    v1v2 = dot(v1vec,v2vec)
    return -x1vec*v1sqr/R**2 - 2*m2*x2vec*v1v2/((m1+m2)*R**2) + forcing - dot(forcing, x1vec)*x1vec/R**2

# acceleration of mass m2
def acc2(x1vec,v1vec,x2vec,v2vec):
    v2sqr = mag2(v2vec)
    v1v2 = dot(v1vec,v2vec)
    return -x2vec*v2sqr/R**2 - 2*m1*x1vec*v1v2/((m1+m2)*R**2)

# Initial conditions
v1vec = cross(omega,x1vec)
v2vec = cross(omega,x2vec)
time = 0.0

# graphics 
if m1overm2 < 1.0:
    m1color = color.red
    m1radius = 0.06
    Show1 = ShowRed
    m2color = color.cyan
    m2radius = 0.12
    Show2 = ShowCyan
elif m1overm2 > 1.0:
    m1color = color.cyan
    m1radius = 0.12
    Show1 = ShowCyan
    m2color = color.red
    m2radius = 0.06
    Show2 = ShowRed
else:
    m1color = color.cyan
    m1radius = 0.09
    Show1 = ShowCyan
    m2color = color.cyan
    m2radiius = 0.09
    Show2 = ShowCyan
m1plus = sphere(pos=x1vec, radius=m1radius, color=m1color, make_trail=Show1, 
    trail_type="points", interval=8, trail_radius=0.015)
m1minus = sphere(pos=-x1vec, radius=m1radius, color=m1color, make_trail=False, 
    trail_type="points", interval=8, trail_radius=0.015)
m2plus = sphere(pos=x2vec, radius=m2radius, color=m2color, make_trail=Show2, 
    trail_type="points", interval=8, trail_radius=0.015)
m2minus = sphere(pos=-x2vec, radius=m2radius, color=m2color, opacity=1.0)
strut1 = cylinder(pos=-x1vec, axis=2*x1vec, radius=0.02, color=m1color) 
strut2 = cylinder(pos=-x2vec, axis=2*x2vec, radius=0.02, color=m2color)
Ball = sphere(pos=vector(0,0,0), radius=R, color=color.white, opacity=0.0)
#plane = box(pos=vector(0,0,0), axis=vector(0,0,1), length=2.2, height=0.05, width=2.2,
#    up=cross(x1vec,v1vec)/mag(v1vec), color=color.red, opacity=0.4)
#info = label(pos=vector(1.3,0,1.1), text="Initial angle ~10 degrees\nOscillation period/Rotation period ~2", height=20, border=4)
#strutz = cylinder(pos=-cross(x1vec,x2vec), axis=2*cross(x1vec,x2vec), radius=0.02, color=color.yellow)

# inertial frame axes
xaxis = arrow(pos=vector(-1.5,0,0), axis=vector(3,0,0), shaftwidth=0.015, color=color.white, opacity=0.25)
yaxis = arrow(pos=vector(0,-1.5,0), axis=vector(0,3,0), shaftwidth=0.015, color=color.white, opacity=0.25)
zaxis = arrow(pos=vector(0,0,-1.5), axis=vector(0,0,3), shaftwidth=0.015, color=color.white, opacity=0.25)

# Angular Velocity
if ShowAV:
    oscale = 1.3*R/mag(omega)
    omegaarrow = arrow(pos=vector(0,0,0), axis=oscale*omega, shaftwidth=0.025, color=color.yellow, opacity=1.0)
    zhat = cross(x1vec,x2vec)/R**2 
# Angular Momentum
if ShowAM:
    Jay = 2*m1*cross(x1vec,v1vec) + 2*m2*cross(x2vec,v2vec)
    Jscale = 1.3*R/mag(Jay)
    Jayarrow = arrow(pos=vector(0,0,0), axis=Jscale*Jay, shaftwidth=0.03, color=color.blue, opacity=1.0) 

# Forcing
if ShowForcing:
    fscale = 0.3*R/mag(forcing)
    forcingarrow1 = arrow(pos=x1vec, axis=fscale*forcing, shaftwidth=0.01, color=color.green, opacity=1.0)
    forcingarrow2 = arrow(pos=-x1vec, axis=fscale*forcing, shaftwidth=0.01, color=color.green, opacity=1.0)
    forcingTorqueArrow  = arrow(pos=Jscale*Jay, axis=fscale*cross(x1vec,forcing), shaftwidth=0.01, color=color.green, opacity=1.0)


# Prepare for evolution 
dt = 0.005  # timestep
dth = dt/2.0           # half timestep
run = False
#scene.caption = " time = {:1.3f}".format(time)

cnt = 0.996
# Evolve using RK4 
while True:
    rate(2.0/dt)
    key = keysdown()
    if 'down' in key:
        run = False
    if 'right' in key:
        run = True
        if dt < 0:
            m1plus.clear_trail()
            m2plus.clear_trail()
        dt = abs(dt)
        dth = 0.5*dt
    if 'left' in key:
        run = True
        if dt > 0:
            m1plus.clear_trail()
            m2plus.clear_trail()
        dt = -abs(dt)
        dth = 0.5*dt
    if run: 
        x1vec1 = x1vec + v1vec*dth
        v1vec1 = v1vec + acc1(x1vec,v1vec,x2vec,v2vec)*dth
        x2vec1 = x2vec + v2vec*dth
        v2vec1 = v2vec + acc2(x1vec,v1vec,x2vec,v2vec)*dth
        # 
        x1vec2 = x1vec + v1vec1*dth
        v1vec2 = v1vec + acc1(x1vec1,v1vec1,x2vec1,v2vec1)*dth
        x2vec2 = x2vec + v2vec1*dth
        v2vec2 = v2vec + acc2(x1vec1,v1vec1,x2vec1,v2vec1)*dth
        #
        x1vec3 = x1vec + v1vec2*dt
        v1vec3 = v1vec + acc1(x1vec2,v1vec2,x2vec2,v2vec2)*dt
        x2vec3 = x2vec + v2vec2*dt
        v2vec3 = v2vec + acc2(x1vec2,v1vec2,x2vec2,v2vec2)*dt
        #
        x1vec4 = x1vec + v1vec3*dt
        v1vec4 = v1vec + acc1(x1vec3,v1vec3,x2vec3,v2vec3)*dt
        x2vec4 = x2vec + v2vec3*dt
        v2vec4 = v2vec + acc2(x1vec3,v1vec3,x2vec3,v2vec3)*dt
        #
        x1vec = (x1vec1 + 2*x1vec2 + x1vec3 + 0.5*x1vec4)/3.0 - 0.5*x1vec
        v1vec = (v1vec1 + 2*v1vec2 + v1vec3 + 0.5*v1vec4)/3.0 - 0.5*v1vec
        x2vec = (x2vec1 + 2*x2vec2 + x2vec3 + 0.5*x2vec4)/3.0 - 0.5*x2vec
        v2vec = (v2vec1 + 2*v2vec2 + v2vec3 + 0.5*v2vec4)/3.0 - 0.5*v2vec
        # print(dot(x1vec, x1vec), dot(x2vec, x2vec), dot(v1vec, v1vec))
        # update mass positions
        m1plus.pos = x1vec
        m1minus.pos = -x1vec
        m2plus.pos = x2vec
        m2minus.pos = -x2vec
        strut1.pos = -x1vec
        strut2.pos = -x2vec
        strut1.axis = 2*x1vec
        strut2.axis = 2*x2vec
        #strutz.pos = -cross(x1vec,x2vec)
        #strutz.axis = 2*cross(x1vec,x2vec)
        #plane.up = cross(x1vec,v1vec)/mag(v1vec)
        if ShowAV:
            zhat = cross(x1vec,x2vec)/R**2
            dzhat = (cross(x1vec,v2vec) + cross(v1vec,x2vec))/R**2
            omega = 0.5*(cross(x1vec,v1vec)/R**2 + cross(x2vec,v2vec)/R**2 + cross(zhat,dzhat))
            omegaarrow.axis = oscale*omega
            #plane.up = cross(x1vec,v1vec)/mag(v1vec)
        if ShowAM:
            Jay = 2*m1*cross(x1vec,v1vec) + 2*m2*cross(x2vec,v2vec)
            Jayarrow.axis = Jscale*Jay 
        if ShowForcing:
            forcingarrow1.axis = fscale*forcing
            forcingarrow1.pos = x1vec
            forcingarrow1.axis = -fscale*forcing
            forcingarrow2.pos = -x1vec
            forcingTorqueArrow.axis = fscale*cross(x1vec,forcing)
            forcingTorqueArrow.pos = Jscale*Jay
            
        #
        #if '1' in key:
        #    omegaarrow.opacity=0.0
        #if '2' in key:
        #    Jayarrow.opacity=0.0
            
        if '1' in key:
            angleup = cnt*angleup
            angleside = cnt*(angleside-pi/2) + pi/2
            scene.camera.pos = cameradistance*vector(cos(angleside)*cos(angleup),sin(angleside)*cos(angleup),sin(angleup))
            scene.camera.axis = -scene.camera.pos
            
        time = time + dt
        #ang = dot(omega,strutz.axis)/mag(omega)
        #scene.caption = " time = {:1.3f}".format(ang)