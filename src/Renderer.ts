import * as THREE from 'three';
import { createFrisbee } from './Frisbee';

export class Renderer {
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private renderer: THREE.WebGLRenderer;

  constructor() {
    const { scene, camera, renderer } = initializeScene();
    this.scene = scene;
    this.camera = camera;
    this.renderer = renderer;
  }

  addObject(object: THREE.Object3D) {
    this.scene.add(object);
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }

  getCamera(): THREE.PerspectiveCamera {
    return this.camera;
  }
}

export function initializeScene() {
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
  const renderer = new THREE.WebGLRenderer();
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

  // Add the frisbee using createFrisbee function
  const frisbee = createFrisbee();
  scene.add(frisbee);

  // Add a floor to the scene
  const floorGeometry = new THREE.PlaneGeometry(50, 50);
  const floorMaterial = new THREE.MeshBasicMaterial({ color: 0xaaaaaa, side: THREE.DoubleSide });
  const floor = new THREE.Mesh(floorGeometry, floorMaterial);
  floor.rotation.x = Math.PI / 2; // Rotate to make it horizontal
  scene.add(floor);

  camera.position.z = 5;

  return { scene, camera, renderer, frisbee, floor };
}