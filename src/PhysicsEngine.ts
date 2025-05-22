import * as THREE from 'three';

export interface FrisbeeState {
  position: THREE.Vector3;
  velocity: THREE.Vector3;
  orientation: [];
  omega: THREE.Vector3; // Angular velocity
  time: number;
}

export class Frisbee {
  area: number;
  mass: number;
  radius: number;
  dragCoefficient: number;
  liftCoefficient: number;
  pitchingMoment: number;
  centerOfLiftRadius: number;

  constructor(area? : number, mass? : number, radius? : number, dragCoefficient? : number, liftCoefficient? : number, centerOfLiftRadius? : number) {
    this.area = area || Math.PI * 0.09; // Frisbee cross-sectional area in m^2
    this.mass = mass || 0.175; // Frisbee mass in kg
    this.radius = radius || 0.1; // Frisbee radius in m
    this.dragCoefficient = dragCoefficient || 0.2; // Drag coefficient
    this.liftCoefficient = liftCoefficient || 0.3; // Lift coefficient
    this.centerOfLiftRadius = centerOfLiftRadius || 0.05; // Center of lift radius in m
  }
}

export class PhysicsEngine {
  private state: FrisbeeState;
  private initalState: FrisbeeState;
  private mass: number;
  private area: number;
  private airDensity: number;

  constructor(initialState: FrisbeeState, mass: number, area: number, airDensity: number) {
    this.state = {
      position: initialState.position.clone(),
      velocity: initialState.velocity.clone(),
      orientation: initialState.orientation.clone(),
      omega: initialState.omega.clone(),
      time: initialState.time
    };
    this.initalState = initialState;
    this.mass = mass;
    this.area = area;
    this.airDensity = airDensity;
  }

  private applyGravity(): THREE.Vector3 {
    return new THREE.Vector3(0, -9.81, 0); // Gravity force in m/s^2
  }

  private computeDragForce(velocity: THREE.Vector3): THREE.Vector3 {
    const speed = velocity.length();
    const dragCoefficient = 0.2; // Placeholder value for drag coefficient
    return velocity.clone().normalize().multiplyScalar(
      -0.5 * this.airDensity * dragCoefficient * this.area * speed * speed
    );
  }

  private computeLiftDragForce({velocity, orientation}: FrisbeeState, windVelocity: THREE.Vector3): THREE.Vector3 {
    
    // Transfrom orientation into the 
    const airVelocity = new THREE.Vector3().subVectors(velocity, windVelocity)

    // const angleOfAttack = velocity.angleTo(orientation.)
    const speed = Math.sqrt(velocity.x * velocity.x + velocity.z * velocity.z)

    const liftCoefficient = 0.3; // Placeholder value for lift coefficient
    const liftDirection = new THREE.Vector3(0, 1, 0); // Placeholder for lift direction
    return liftDirection.multiplyScalar(
      0.5 * this.airDensity * liftCoefficient * this.area * speed * speed
    );
  }

  private handleCollision(): void {
    if (this.state.position.y <= 0) {
      this.state.position.y = 0; // Prevent sinking below the floor

      // Apply friction force
      const frictionCoefficient = 0.8; // Coefficient of friction
      const normalForce = this.mass * 9.81; // Normal force due to gravity
      const frictionForceMagnitude = frictionCoefficient * normalForce;

      // Compute friction force direction (opposite to velocity)
      const horizontalVelocity = new THREE.Vector3(this.state.velocity.x, 0, this.state.velocity.z);
      if (horizontalVelocity.length() > 0) {
        const frictionForce = horizontalVelocity.clone().normalize().multiplyScalar(-frictionForceMagnitude);

        // Update velocity with friction
        const frictionAcceleration = frictionForce.divideScalar(this.mass);
        this.state.velocity.add(frictionAcceleration.multiplyScalar(0.016)); // Assuming 60 FPS

        // Stop the frisbee if the velocity is very small
        if (horizontalVelocity.length() < 0.1) {
          this.state.velocity.x = 0;
          this.state.velocity.z = 0;
        }
      }

      // Stop vertical velocity
      this.state.velocity.y = 0;
    }
  }

  step(dt: number) {
    // Compute forces
    const gravity = this.applyGravity();
    const liftDragForce = this.computeLiftDragForce(this.state);

    // Sum forces
    const totalForce = gravity.clone().add(liftDragForce);

    // Compute acceleration: a = F / m
    const acceleration = totalForce.divideScalar(this.mass);

    // Update velocity: v = v + a * dt
    this.state.velocity.add(acceleration.multiplyScalar(dt));

    // Update position: x = x + v * dt
    this.state.position.add(this.state.velocity.clone().multiplyScalar(dt));

    // Handle collisions
    this.handleCollision();

    // Update time
    this.state.time += dt;
  }

  resetState(initialState?: FrisbeeState) {
    initialState = initialState || this.initalState;
    console.log(initialState)
    this.state.position.copy(initialState.position);
    this.state.velocity.copy(initialState.velocity);
    this.state.orientation.copy(initialState.orientation);
    this.state.omega.copy(initialState.omega);
    this.state.time = initialState.time;
  }

  setInitialConditions(initialConditions: { velocity: number; spinRate: number; launchAngle: number, orientation?: [],  }) {
    this.initalState.position.set(0, 1, 0);   // Default initial position
    this.initalState.velocity.set(
      initialConditions.velocity * Math.cos((initialConditions.launchAngle * Math.PI) / 180),
      initialConditions.velocity * Math.sin((initialConditions.launchAngle * Math.PI) / 180),
      0
    );
    this.initalState.omega.set(0, 0, initialConditions.spinRate);
    if (initialConditions.orientation) {
      this.initalState.orientation.copy(initialConditions.orientation);
    }
  }

  getState(): FrisbeeState {
    return this.state;
  }
}

export function initializePhysics(frisbee: THREE.Mesh) {
  const initialState: FrisbeeState = {
    position: new THREE.Vector3(0, 1, 0),
    velocity: new THREE.Vector3(5, 5, 0),
    orientation: new THREE.Vector3(),
    omega: new THREE.Vector3(0, 0, 10),
    time: 0,
  };

  const mass = 0.175; // Frisbee mass in kg
  const area = Math.PI * 0.09; // Frisbee cross-sectional area in m^2
  const airDensity = 1.225; // Air density in kg/m^3

  return new PhysicsEngine(initialState, mass, area, airDensity);
}