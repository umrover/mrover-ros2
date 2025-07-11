<template>
  <div class="wrapper">
    <div id="threejs"></div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex   
// import threeSetup, { fk, ik } from '../rover_three.js'
// if confirmed working, remove rover_three.js in /src

import * as THREE from 'three'
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader.js'
import { TrackballControls } from 'three/addons/controls/TrackballControls.js'
import type { Joint, IKTarget } from '../types/threejs.ts'


function threeSetup(containerId: string) {
  const container = document.getElementById(containerId)
  if(!container) {
    console.error(`Container with id ${containerId} not found`)
    return
  }

  

  const canvas = {
    width: container.clientWidth,
    height: container.clientHeight
  }

  console.log(`Canvas size: ${canvas.width}x${canvas.height}`)

  const scene = new THREE.Scene()
  const camera = new THREE.PerspectiveCamera(75, canvas.width / canvas.height, 0.1, 1000)
  camera.up.set(0, 0, 1)

  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5)
  directionalLight.position.set(1, 2, 3)
  scene.add(directionalLight)

  const renderer = new THREE.WebGLRenderer()
  renderer.setSize(canvas.width, canvas.height, true)
  container.appendChild(renderer.domElement)

  scene.background = new THREE.Color(0xeeeeee)

  const controls = new TrackballControls(camera, renderer.domElement)
  controls.rotateSpeed = 1.0
  controls.zoomSpeed = 1.2
  controls.panSpeed = 0.8
  controls.noZoom = false
  controls.noPan = false
  controls.staticMoving = true
  controls.dynamicDampingFactor = 0.3

  const joints = [
    { name: 'chassis', file: 'rover_chassis.fbx', translation: [0.164882, -0.200235, 0.051497], rotation: [0, 0, 0] },
    { name: 'a', file: 'arm_a.fbx', translation: [0.034346, 0, 0.049024], rotation: [0, 0, 0] },
    { name: 'b', file: 'arm_b.fbx', translation: [0.534365, 0, 0.009056], rotation: [0, 0, 0] },
    { name: 'c', file: 'arm_c.fbx', translation: [0.546033, 0, 0.088594], rotation: [0, 0, 0] },
    { name: 'd', file: 'arm_d.fbx', translation: [0.044886, 0, 0], rotation: [0, 0, 0] },
    { name: 'e', file: 'arm_e.fbx', translation: [0.044886, 0, 0], rotation: [0, 0, 0] }
  ]

  const loader = new FBXLoader()
  for (let i = 0; i < joints.length; ++i) {
    loader.load('/meshes/' + joints[i].file, (fbx) => {
      fbx.name = joints[i].name
      fbx.traverse((child) => {
        if (child.isMesh) {
          child.name = joints[i].name
          child.material = new THREE.MeshStandardMaterial({ color: (joints[i].name === 'd' || joints[i].name === 'e') ? 0x00ff00 : 0xffffff })
          child.scale.set(1, 1, 1)
          child.position.set(0, 0, 0)
          scene.add(child)
        }
      })
    })
  }

  camera.position.set(1.25, 1.25, 1.25)
  camera.lookAt(new THREE.Vector3(0, 0, 0))

  const targetCube = new THREE.Mesh(
    new THREE.BoxGeometry(0.1, 0.1, 0.1),
    new THREE.MeshBasicMaterial({ color: 0x00ff00 })
  )
  scene.add(targetCube)

  const setSize = () => {
    const width = container.clientWidth
    const height = container.clientHeight
    camera.aspect = width / height
    camera.updateProjectionMatrix()
    renderer.setSize(width, height, true)
  }
  setSize()

  const resizeObserver = new ResizeObserver(() => {
    setSize()
    // console.log(`ResizeObserver: Canvas size updated to ${container.clientWidth}x${container.clientHeight}`)
  })
  resizeObserver.observe(container)

  function animate() {
    requestAnimationFrame(animate)
    controls.update()
    renderer.render(scene, camera)
  }

  animate()

  // return {
  //   fk: (positions) => fk(positions, scene, joints),
  //   ik: (target) => ik(target, targetCube)
  // }
}

function fk(positions: number[], scene: THREE.Scene, joints: Joint[]) {
  const cumulativeMatrix = new THREE.Matrix4()
  cumulativeMatrix.makeTranslation(new THREE.Vector3(0, 0, 0.439675)) // base_link offset

  for (let i = 0; i < joints.length; ++i) {
    const mesh = scene.getObjectByName(joints[i].name)
    if (!mesh) continue

    const localMatrix = new THREE.Matrix4()
    const rotationAngle = positions[i]

    if (joints[i].name === 'chassis') {
      localMatrix.makeTranslation(0, rotationAngle, 0)
    } else {
      localMatrix.makeRotationY(rotationAngle)
    }

    const offset = new THREE.Vector3().fromArray(joints[i].translation)
    localMatrix.setPosition(
      new THREE.Vector3().setFromMatrixPosition(localMatrix).add(offset)
    )

    mesh.matrixAutoUpdate = false
    mesh.matrix = cumulativeMatrix.clone()

    cumulativeMatrix.multiply(localMatrix)
  }
}

function ik(target: IKTarget, targetCube: THREE.Mesh) {
  const quaternion = new THREE.Quaternion(...target.quaternion)
  targetCube.position.set(...target.position)
  targetCube.setRotationFromQuaternion(quaternion)
}

export default defineComponent({
  data() {
    return {
      threeScene: null as ReturnType<typeof threeSetup> | null,
      temp_positions: ['base', 'a', 'b', 'c', 'd', 'e'],
      positions: [] as number[]
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  mounted() {
    this.threeScene = threeSetup('threejs')
  },

  watch: {
    message(msg: { type: string; position?: number[]; target?: IKTarget }) {
      if (!this.threeScene) return

      if (msg.type === 'fk' && msg.position) {
        const sanitized = msg.position.map((x) => isNaN(x) ? 0 : x)
        fk(sanitized, this.threeScene.scene, this.threeScene.joints)
      } else if (msg.type === 'ik' && msg.target) {
        ik(msg.target, this.threeScene.targetCube)
      }
    }
  }
})
</script>

<style scoped>
.wrapper {
  margin: 0;
  padding: 0;
  height: 100%;
  width: 100%;
}

.header {
  text-align: center;
}

#threejs {
  height: 100%;
  width: 100%;
  margin: 0;
  padding: 0;
}

#threejs > canvas {
  display: block;
  width: 100% !important;
  height: 100% !important;
}
</style>
