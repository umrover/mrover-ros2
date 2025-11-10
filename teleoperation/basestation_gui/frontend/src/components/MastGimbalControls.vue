<template>
  <div class='d-flex flex-column' style="flex: 0 0 auto;">
    <h4 class="mb-2">Gimbal Controls</h4>
    <div class="d-flex flex-column gap-3 align-items-center">
      <div class="d-flex gap-2 align-items-center">
        <span class="fw-bold small text-nowrap">Pitch: {{ pitchDegrees }}°</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('pitch', -10)">-10°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('pitch', -5)">-5°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('pitch', -1)">-1°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('pitch', 1)">+1°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('pitch', 5)">+5°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('pitch', 10)">+10°</button>
        </div>
      </div>
      <div class="d-flex gap-2 align-items-center">
        <span class="fw-bold small text-nowrap">Yaw: {{ yawDegrees }}°</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('yaw', -10)">-10°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('yaw', -5)">-5°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('yaw', -1)">-1°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('yaw', 1)">+1°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('yaw', 5)">+5°</button>
          <button class="btn btn-outline-primary px-2 py-1" @click="adjustGimbal('yaw', 10)">+10°</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang='ts' setup>
import { computed } from 'vue'
import { mastAPI } from '@/utils/mastAPI'
import { useWebsocketStore } from '@/stores/websocket'
import type { JointStateMessage } from '@/types/websocket'

const websocketStore = useWebsocketStore()

const gimbalJointState = computed((): JointStateMessage | null => {
  const messages = websocketStore.messages['mast']
  if (!messages) return null
  const msg = messages.find((msg) => msg.type === 'gimbal_joint_state')
  return msg ? (msg as JointStateMessage) : null
})

const pitchRadians = computed((): number => {
  const state = gimbalJointState.value
  if (!state || !state.name || !state.position) return 0
  const pitchIndex = state.name.indexOf('pitch')
  return pitchIndex >= 0 ? state.position[pitchIndex] : 0
})

const yawRadians = computed((): number => {
  const state = gimbalJointState.value
  if (!state || !state.name || !state.position) return 0
  const yawIndex = state.name.indexOf('yaw')
  return yawIndex >= 0 ? state.position[yawIndex] : 0
})

const pitchDegrees = computed((): string => (pitchRadians.value * 180 / Math.PI).toFixed(1))
const yawDegrees = computed((): string => (yawRadians.value * 180 / Math.PI).toFixed(1))

const adjustGimbal = async (joint: 'pitch' | 'yaw', adjustment: number): Promise<void> => {
  try {
    await mastAPI.adjustGimbal(joint, adjustment, false)
  } catch (error) {
    console.error('Failed to adjust gimbal:', error)
  }
}
</script>
