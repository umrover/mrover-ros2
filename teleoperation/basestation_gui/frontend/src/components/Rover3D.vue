<template>
  <div class="d-flex flex-wrap gap-1 mb-2">
    <button
      type="button"
      class="btn btn-sm btn-outline-control border-2 flex-grow-1"
      @click="set_camera_type('default')">
      Default
    </button>
    <button
      type="button"
      class="btn btn-sm btn-outline-control border-2 flex-grow-1"
      @click="set_camera_type('follow')">
      Follow
    </button>
    <button
      type="button"
      class="btn btn-sm btn-outline-control border-2 flex-grow-1"
      @click="set_camera_type('arm')">
      Arm
    </button>
    <button
      type="button"
      class="btn btn-sm btn-outline-control border-2 flex-grow-1"
      @click="set_camera_type('full arm')">
      Full Arm
    </button>
    <button
      type="button"
      class="btn btn-sm btn-outline-control border-2 flex-grow-1"
      @click="set_camera_type('side arm')">
      Side Arm
    </button>
    <button
      type="button"
      class="btn btn-sm btn-outline-control border-2 flex-grow-1"
      @click="set_camera_type('top')">
      Top Down
    </button>
    <button
      type="button"
      class="btn btn-sm btn-outline-control border-2 flex-grow-1"
      @click="set_camera_type('bottom')">
      Bottom Up
    </button>
    <button
      type="button"
      class="btn btn-sm btn-outline-secondary border-2 flex-grow-1"
      @click="updateCostMapGrid()">
      Test
    </button>
  </div>

  <canvas class="webgl p-0 h-100 w-100"></canvas>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex
// @ts-expect-error shut up ts
import threeSetup, { updatePose } from '../rover_three.js'
import type { WebSocketState } from '../types/websocket.js'

const jointNameMap: Record<string, string> = {
  joint_a: 'chassis_to_arm_a',
  joint_b: 'arm_a_to_arm_b',
  joint_c: 'arm_b_to_arm_c',
  joint_de_pitch: 'arm_c_to_arm_d',
  joint_de_roll: 'arm_d_to_arm_e',
  gripper: 'gripper_link',
}


onBeforeUnmount(() => {
  if (threeScene.value) {
    threeScene.value()
  }
})

const armMessage = computed(() => messages.value['arm'])
const contextMessage = computed(() => messages.value['context'])

watch(armMessage, (msg: unknown) => {
  if (!msg || typeof msg !== 'object') return

  if ('type' in msg && msg.type === 'arm_state') {
    const typedMsg = msg as ControllerStateMessage
    const joints = typedMsg.names.map((name: string, index: number) => {
      const urdfName = jointNameMap[name] || name
      let position = typedMsg.positions[index] ?? 0

      if (urdfName === 'chassis_to_arm_a') {
        position = position * -100 + 40
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
  },

  mounted() {
    this.threeScene = threeSetup('threejs')
  },

  computed: {
    ...mapState('websocket', {
      armMessage: (state: WebSocketState) => state.messages['arm'],
    }),
  },

  watch: {
    armMessage(msg) {
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

      console.log(joints)
      updatePose(joints)

      // else if (msg.type === 'ik') {
      //   this.threeScene.ik(msg.target)
      // }
    },
  },
})
</script>

<style scoped>
</style>
