
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { PhysicsEngine } from './PhysicsEngine';

export interface SimulationParameters {
  velocity: number;
  spinRate: number;
  launchAngle: number;
}

export function initializeControls(camera: THREE.PerspectiveCamera, renderer: THREE.WebGLRenderer, physicsEngine: PhysicsEngine) {
  const orbitControls = new OrbitControls(camera, renderer.domElement);
  orbitControls.enableDamping = true; // Smooth camera movement
  orbitControls.dampingFactor = 0.05;
  orbitControls.update();

  // Enable movement in space using keyboard controls
  const movementSpeed = 0.1;
  const movement = { forward: false, backward: false, left: false, right: false, up: false, down: false };

  document.addEventListener('keydown', (event) => {
    switch (event.key) {
      case 'w':
        movement.forward = true;
        break;
      case 's':
        movement.backward = true;
        break;
      case 'a':
        movement.left = true;
        break;
      case 'd':
        movement.right = true;
        break;
      case 'q':
        movement.up = true;
        break;
      case 'e':
        movement.down = true;
        break;
    }
  });

  document.addEventListener('keyup', (event) => {
    switch (event.key) {
      case 'w':
        movement.forward = false;
        break;
      case 's':
        movement.backward = false;
        break;
      case 'a':
        movement.left = false;
        break;
      case 'd':
        movement.right = false;
        break;
      case 'q':
        movement.up = false;
        break;
      case 'e':
        movement.down = false;
        break;
    }
  })

  document.addEventListener('keypress', (event) => {
    if (event.key === 'r') {
      physicsEngine.resetState();
      console.log('Physics engine state reset');
    }
  });
  function updateCameraPosition() {
    if (movement.forward) camera.position.z -= movementSpeed;
    if (movement.backward) camera.position.z += movementSpeed;
    if (movement.left) camera.position.x -= movementSpeed;
    if (movement.right) camera.position.x += movementSpeed;
    if (movement.up) camera.position.y += movementSpeed;
    if (movement.down) camera.position.y -= movementSpeed;
  }

  return { orbitControls, updateCameraPosition };
}