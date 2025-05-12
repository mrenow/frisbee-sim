import { PhysicsEngine } from "./PhysicsEngine";
import * as THREE from 'three';
import * as dat from 'dat.gui';

export function initializeUI(physicsEngine: PhysicsEngine, camera: THREE.PerspectiveCamera) {
  const readoutDiv = document.createElement('div');
  readoutDiv.style.position = 'absolute';
  readoutDiv.style.top = '10px';
  readoutDiv.style.left = '10px';
  readoutDiv.style.backgroundColor = 'rgba(255, 255, 255, 0.8)';
  readoutDiv.style.padding = '10px';
  readoutDiv.style.borderRadius = '5px';
  readoutDiv.style.fontFamily = 'Arial, sans-serif';
  readoutDiv.style.fontSize = '14px';
  readoutDiv.style.zIndex = '1000';
  document.body.appendChild(readoutDiv);

  function updateReadouts() {
    const state = physicsEngine.getState();
    readoutDiv.innerHTML = `
      <strong>Frisbee Parameters:</strong><br>
      Position: (${state.position.x.toFixed(2)}, ${state.position.y.toFixed(2)}, ${state.position.z.toFixed(2)})<br>
      Velocity: (${state.velocity.x.toFixed(2)}, ${state.velocity.y.toFixed(2)}, ${state.velocity.z.toFixed(2)})<br>
      <strong>Camera Position:</strong><br>
      (${camera.position.x.toFixed(2)}, ${camera.position.y.toFixed(2)}, ${camera.position.z.toFixed(2)})
    `;
  }

  setInterval(updateReadouts, 100); // Update readouts every 100ms

  // Add dat.GUI for initial conditions
  const gui = new dat.GUI();
  const initialConditions = {
    velocity: 5,
    spinRate: 10,
    launchAngle: 45,
  };

  // Update sliders to call setInitialConditions on the physics engine
  gui.add(initialConditions, 'velocity', 1, 30).name('Velocity (m/s)').onChange((value) => {
    initialConditions.velocity = value;
    console.log('Velocity changed:', value);
    physicsEngine.setInitialConditions(initialConditions);
  });

  gui.add(initialConditions, 'spinRate', 0, 10).name('Spin Rate (rev/s)').onChange((value) => {
    initialConditions.spinRate = value;
    physicsEngine.setInitialConditions(initialConditions);
  });

  gui.add(initialConditions, 'launchAngle', 0, 90).name('Launch Angle (°)').onChange((value) => {
    initialConditions.launchAngle = value;
    physicsEngine.setInitialConditions(initialConditions);
  });
}