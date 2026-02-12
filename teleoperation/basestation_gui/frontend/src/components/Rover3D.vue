<template>
  <div class="d-flex gap-3 p-2">
    <div class="dropdown">
      <button
        class="btn btn-success dropdown-toggle"
        type="button"
        id="cameraDropdown"
        data-bs-toggle="dropdown"
        aria-expanded="false"
      >
        Camera: {{ camera_type === 'default' ? 'Default' : camera_type === 'follow' ? 'Follow' : camera_type === 'arm' ? 'Arm' : camera_type === 'full arm' ? 'Full Arm' : camera_type === 'side arm' ? 'Side Arm' : 'Top Down' }}
      </button>
      <ul class="dropdown-menu" aria-labelledby="cameraDropdown">
        <li><a class="dropdown-item" :class="{ 'active': camera_type === 'default' }" @click="change_camera('default')">Default</a></li>
        <li><a class="dropdown-item" :class="{ 'active': camera_type === 'follow' }" @click="change_camera('follow')">Follow</a></li>
        <li><a class="dropdown-item" :class="{ 'active': camera_type === 'arm' }" @click="change_camera('arm')">Arm</a></li>
        <li><a class="dropdown-item" :class="{ 'active': camera_type === 'full arm' }" @click="change_camera('full arm')">Full Arm</a></li>
        <li><a class="dropdown-item" :class="{ 'active': camera_type === 'side arm' }" @click="change_camera('side arm')">Side Arm</a></li>
        <li><a class="dropdown-item" :class="{ 'active': camera_type === 'top' }" @click="change_camera('top')">Top Down</a></li>
      </ul>
    </div>

    <button
        type="button"
        class="btn btn-sm btn-light border"
        @click="toggleCostMapGridVisibility()">
          Toggle Cost Map
    </button>
  </div>

  <canvas class="webgl p-0 h-100 w-100"></canvas>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage, OccupancyGridMessage } from '@/types/websocket'
import threeSetup, { updatePose, updateIKTarget, set_camera_type, updateCostMapGrid, toggleCostMapGridVisibility} from '../rover_three.js'
import type { NavMessage } from '@/types/coordinates.js'
import { round } from 'three/tsl'

interface ArmIKMessage {
  type: 'ik_target'
  target: {
    pose: {
      position: {
        x: number
        y: number
        z: number
      }
    }
  }
}

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const threeScene = ref<(() => void) | null>(null)

const jointNameMap: Record<string, string> = {
  joint_a: 'chassis_to_arm_a',
  joint_b: 'arm_a_to_arm_b',
  joint_c: 'arm_b_to_arm_c',
  joint_de_pitch: 'arm_c_to_arm_d',
  joint_de_roll: 'arm_d_to_arm_e',
  gripper: 'gripper_link', // not implemented lol
}

onMounted(() => {
  threeScene.value = threeSetup()
  websocketStore.setupWebSocket('nav')
})

onBeforeUnmount(() => {
  if (threeScene.value) {
    threeScene.value()
  }
  websocketStore.closeWebSocket('nav')
})

const armMessage = computed(() => messages.value['arm'])
const driveMessage = computed(() => messages.value['drive'])
const navMessage = computed(() => messages.value['nav'])

watch(armMessage, (msg: unknown) => {
  if (!msg || typeof msg !== 'object') return

  if ('type' in msg && msg.type === 'arm_state') {
    const typedMsg = msg as ControllerStateMessage
    const joints = typedMsg.names.map((name: string, index: number) => {
      const urdfName = jointNameMap[name] || name
      let position = typedMsg.positions[index] ?? 0

      if (urdfName === 'chassis_to_arm_a') {
        position = position * -100 + 40 // scale from m to cm
      }

      return {
        name: urdfName,
        position,
      }
    })

    updatePose(joints)
  } else if ('type' in msg && msg.type === 'ik_target') {
    const typedMsg = msg as ArmIKMessage
    if (typedMsg.target.pose && typedMsg.target.pose.position) {
      const position = {
        x: typedMsg.target.pose.position.x * 100,
        y: typedMsg.target.pose.position.z * 100,
        z: typedMsg.target.pose.position.y * -100 + 20,
      }
      updateIKTarget(position)
    }
  }
})

let roverPos = {
  latitude: 0.0,
  longitude: 0.0
}

watch(driveMessage, (msg: unknown) => {
  if (!msg || typeof msg !== 'object') return

  if ('data' in msg){
    const typedMsg = msg as OccupancyGridMessage
    
    // let costMapPos = {
    //   latitude: typedMsg.info.origin.position.x,
    //   longitude: typedMsg.info.origin.position.y
    // }

    const default_offset = 40

    let offsetPos = {
      x: Math.floor((roverPos.longitude + 83.70967) * 10000 - (typedMsg.info.origin.position.x + 30)) + 40,
      y: Math.floor((roverPos.latitude - 42.29319) * 10000 - (typedMsg.info.origin.position.y + 30)) + 40
    }


    let processed_data = []
    for(let i = 0, k = 0; i < offsetPos.y; ++i){
      for(let j = 0; j < offsetPos.x; ++j, ++k){
        processed_data[k] = typedMsg.data[((i + offsetPos.y) * 120) + j + offsetPos.x]
      }
    }
    
    updateCostMapGrid(processed_data)
    // console.log(roverPos)
    // console.log(typedMsg.info.origin.position)
    // console.log(offsetPos)
    // console.log(typedMsg.data)
    // console.log(processed_data)
  }
})

watch(navMessage, msg => {
  // console.log("got")
if (!msg) return
  const navMsg = msg as NavMessage
  if (navMsg.type === 'gps_fix') {
    roverPos.latitude = navMsg.latitude
    roverPos.longitude = navMsg.longitude
    // rover_latitude_deg.value = navMsg.latitude
    // rover_longitude_deg.value = navMsg.longitude
    // rover_altitude.value = navMsg.altitude
    // rover_status.value = navMsg.status.status
  }
})


const camera_type = ref('default')

function change_camera(new_mode: string){
  set_camera_type(new_mode)
  camera_type.value = new_mode
}
</script>
