<template>
  <canvas class="webgl p-0 h-100 w-100"></canvas>
</template>

<script lang="ts" setup>
import { defineComponent, ref, computed, watch, onMounted } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import threeSetup, { updatePose, updateIKTarget } from '../rover_three.js'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const threeScene = ref(null)

const jointNameMap: Record<string, string> = {
  joint_a: 'chassis_to_arm_a',
  joint_b: 'arm_a_to_arm_b',
  joint_c: 'arm_b_to_arm_c',
  joint_de_pitch: 'arm_c_to_arm_d',
  joint_de_roll: 'arm_d_to_arm_e',
  gripper: 'gripper_link', // not implemented lol
}

onMounted(() => {
  threeScene.value = threeSetup('threejs')
})

const armMessage = computed(() => messages.value['arm'])

watch(armMessage, (msg) => {
  if (!msg) return

  if (msg.type === 'fk') {
    const joints = msg.name.map((name: string, index: number) => {
      const urdfName = jointNameMap[name] || name
      let position = msg.position[index]

      if (urdfName === 'chassis_to_arm_a') {
        position = position * -100 + 40 // scale from m to cm
      }

      return {
        name: urdfName,
        position,
      }
    })

    updatePose(joints)
  } else if (msg.type === 'ik_target') {
    console.log(msg)
    if (msg.target.pose && msg.target.pose.position) {
      const position = {
        x: msg.target.pose.position.x * 100,
        y: msg.target.pose.position.z * 100,
        z: msg.target.pose.position.y * -100 + 20,
      }
      console.log(position)
      updateIKTarget(position)
    }
  }
})
</script>
