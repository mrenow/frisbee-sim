import './style.css';
import { initializeScene } from './Renderer';
import { initializePhysics } from './PhysicsEngine';
import { initializeControls } from './Controls';
import { initializeUI, getInitialConditions } from './UI';

// Initialize the scene, physics, controls, and UI
const { scene, camera, renderer, frisbee, floor } = initializeScene();
const physicsEngine = initializePhysics(frisbee);
const { orbitControls, updateCameraPosition } = initializeControls(camera, renderer, physicsEngine);
initializeUI(physicsEngine, camera);

// Animation loop
function animate() {
  requestAnimationFrame(animate);

  // Update physics
  const dt = 0.016; // Time step in seconds (60 FPS)
  physicsEngine.step(dt);
  const state = physicsEngine.getState();

  // Update frisbee mesh position and orientation
  frisbee.position.copy(state.position);
  frisbee.quaternion.copy(state.orientation);

  // Update camera position and controls
  updateCameraPosition();
  orbitControls.update();

  // Render the scene
  renderer.render(scene, camera);
}

animate();
