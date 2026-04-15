<template>
  <div class="flex flex-col w-full h-full">
    <div class="flex flex-wrap gap-1 mb-2 shrink-0">
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        @click="set_camera_type('default')">
        Default
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        @click="set_camera_type('follow')">
        Follow
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        @click="set_camera_type('arm')">
        Arm
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        @click="set_camera_type('full arm')">
        Full Arm
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        @click="set_camera_type('side arm')">
        Side Arm
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        @click="set_camera_type('top')">
        Top Down
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        @click="set_camera_type('bottom')">
        Bottom Up
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-secondary grow"
        @click="updateCostMapGrid()">
        Test
      </button>
    </div>
    <canvas class="webgl grow min-h-0 w-full"></canvas>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import threeSetup, { updatePose, updateIKTarget, set_camera_type, updateCostMapGrid} from '../rover_three.js'

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

interface CostmapMessage {
  type: 'costmap'
}

const websocketStore = useWebsocketStore()

const threeScene = ref<(() => void) | null>(null)

const jointNameMap: Record<string, string> = {
  joint_a: 'chassis_to_arm_a',
  joint_b: 'arm_a_to_arm_b',
  joint_c: 'arm_b_to_arm_c',
  joint_de_pitch: 'arm_c_to_arm_d',
  joint_de_roll: 'arm_d_to_arm_e',
  gripper: 'gripper_link',
}

onMounted(() => {
  threeScene.value = threeSetup()
})

onBeforeUnmount(() => {
  if (threeScene.value) {
    threeScene.value()
  }
})

websocketStore.onMessage<ControllerStateMessage>('arm', 'arm_state', (msg) => {
  if (!Array.isArray(msg.names)) return
  const joints = msg.names.map((name: string, index: number) => {
    const urdfName = jointNameMap[name] || name
    let position = msg.positions[index] ?? 0

    if (urdfName === 'chassis_to_arm_a') {
      position = position * -100 + 40
    }

    return {
      name: urdfName,
      position,
    }
  })

  updatePose(joints)
})

websocketStore.onMessage<ArmIKMessage>('arm', 'ik_target', (msg) => {
  if (msg.target.pose && msg.target.pose.position) {
    const position = {
      x: msg.target.pose.position.x * 100,
      y: msg.target.pose.position.z * 100,
      z: msg.target.pose.position.y * -100 + 20,
    }
    updateIKTarget(position)
  }
})

websocketStore.onMessage<CostmapMessage>('context', 'costmap', () => {
  updateCostMapGrid()
})
</script>
