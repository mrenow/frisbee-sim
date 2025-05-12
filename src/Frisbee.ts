import * as THREE from 'three';

export function createFrisbee() {
    const height = 0.05
    const frisbeeGeometry = new THREE.CylinderGeometry(0.3, 0.3, height, 32, 1, true);
    const frisbeeMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000, wireframe: true });
    const frisbee = new THREE.Mesh(frisbeeGeometry, frisbeeMaterial);

    // Make the frisbee more frisbee-shaped by adding a slight curvature
    const vertices = frisbeeGeometry.attributes.position;
    let vertex = new THREE.Vector3();
    for (let i = 0; i < vertices.count; i++) {
        vertex.fromBufferAttribute(vertices, i);

        const radialDistance = Math.sqrt(vertex.x * vertex.x + vertex.z * vertex.z);

        vertex = vertex.multiplyScalar((radialDistance - vertex.y) / radialDistance);
        vertex.y += height/2
        vertices.setXYZ(i, vertex.x, vertex.y, vertex.z);
        
    }
    frisbeeGeometry.attributes.position.needsUpdate = true;

    return frisbee;
}