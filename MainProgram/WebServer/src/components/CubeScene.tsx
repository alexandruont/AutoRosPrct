// MeshModel.tsx
import { useFrame, useLoader, extend } from '@react-three/fiber'
import { OBJLoader, OrbitControls } from 'three-stdlib'
import { shaderMaterial } from '@react-three/drei'
import { useRef } from 'react'
import * as THREE from 'three'
import React from 'react'

export default function MeshModel() {
  const obj = useLoader(OBJLoader, './res/models/bearing.obj')
  obj.visible = true
  const ref = useRef<THREE.Group>(null)

  useFrame(() => {
    obj.rotation.y += 0.01
  })

  const MyShaderMaterial = shaderMaterial(
    { uColor: new THREE.Color('orange') },
    ` // vertex.glsl
      varying vec3 vPos;
      void main() {
        vPos = position;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
      }
    `,
    ` // fragment.glsl
      uniform vec3 uColor;
      varying vec3 vPos;
      void main() {
        float dist = length(vPos);
        float darken = smoothstep(1.0, 0.0, dist); // darker on edges
        gl_FragColor = vec4(uColor * darken, 1.0);
      }
    `
  )
  
  extend({ MyShaderMaterial })

  return <>
    <primitive object={obj} ref={ref} scale={0.5} position={[0, 0, 0]}>
      <meshStandardMaterial attach="material" color="orange" />
    </primitive>
    <ambientLight intensity={0.3} />
    <pointLight position={[5, 5, 5]} intensity={1} />
    <directionalLight position={[-3, 5, 5]} intensity={0.5} />
  </>
}
