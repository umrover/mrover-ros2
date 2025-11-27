<template>
  <canvas class="webgl p-0 h-100 w-100"></canvas>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage } from '@/types/websocket'
import threeSetup, { updatePose, updateIKTarget } from '../rover_three.js'

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
})

onBeforeUnmount(() => {
  if (threeScene.value) {
    threeScene.value()
  }
})

const armMessage = computed(() => messages.value['arm'])

watch(armMessage, (msg: unknown) => {
  if (!msg || typeof msg !== 'object') return

  if ('type' in msg && msg.type === 'arm_state') {
    const typedMsg = msg as ControllerStateMessage
    const joints = typedMsg.name.map((name: string, index: number) => {
      const urdfName = jointNameMap[name] || name
      let position = typedMsg.position[index]

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
</script>
