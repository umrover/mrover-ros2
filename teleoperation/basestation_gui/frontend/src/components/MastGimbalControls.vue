<template>
  <div class="gimbal-container">
    <div class="cross-layout">
      <h4 class="m-0 pt-2 font-monospace align-self-start justify-self-start" style="grid-area: title;">Gimbal</h4>
      <div class="btn-group-vertical btn-group-sm arm top">
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('pitch', 10)"
        >
          10
        </button>
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('pitch', 5)"
        >
          5
        </button>
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('pitch', 1)"
        >
          1
        </button>
      </div>
      <div class="pe-1 pt-2 align-self-start justify-self-start" style="grid-area: readings;">
        <div class="small fw-semibold font-monospace">P: {{ pitchDegrees }}</div>
        <div class="small fw-semibold font-monospace">Y: {{ yawDegrees }}</div>
      </div>
      <div class="btn-group btn-group-sm arm left">
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('yaw', -10)"
        >
          10
        </button>
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('yaw', -5)"
        >
          5
        </button>
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('yaw', -1)"
        >
          1
        </button>
      </div>
      <div class="cross-center"></div>
      <div class="btn-group btn-group-sm arm right">
        <button class="btn btn-outline-primary" @click="adjustGimbal('yaw', 1)">
          1
        </button>
        <button class="btn btn-outline-primary" @click="adjustGimbal('yaw', 5)">
          5
        </button>
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('yaw', 10)"
        >
          10
        </button>
      </div>
      <div class="btn-group-vertical btn-group-sm arm bottom">
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('pitch', -1)"
        >
          1
        </button>
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('pitch', -5)"
        >
          5
        </button>
        <button
          class="btn btn-outline-primary"
          @click="adjustGimbal('pitch', -10)"
        >
          10
        </button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { computed } from 'vue'
import { mastAPI } from '@/utils/mastAPI'
import { useWebsocketStore } from '@/stores/websocket'
import type { JointStateMessage } from '@/types/websocket'

const websocketStore = useWebsocketStore()

const gimbalJointState = computed((): JointStateMessage | null => {
  const messages = websocketStore.messages['mast']
  if (!messages || !Array.isArray(messages)) return null
  const msg = messages.find((msg: { type: string }) => msg.type === 'gimbal_joint_state')
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

const pitchDegrees = computed((): string =>
  ((pitchRadians.value * 180) / Math.PI).toFixed(0),
)
const yawDegrees = computed((): string =>
  ((yawRadians.value * 180) / Math.PI).toFixed(0),
)

const adjustGimbal = async (
  joint: 'pitch' | 'yaw',
  adjustment: number,
): Promise<void> => {
  try {
    await mastAPI.adjustGimbal(joint, adjustment, false)
  } catch (error) {
    console.error('Failed to adjust gimbal:', error)
  }
}
</script>

<style scoped>
.gimbal-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  padding: 8px;
}

.cross-layout {
  display: grid;
  grid-template-areas:
    'title top readings'
    'left center right'
    '. bottom .';
  grid-template-columns: auto 32px auto;
  grid-template-rows: auto 32px auto;
  gap: 4px;
  align-items: center;
  justify-items: center;
}



.arm.top {
  grid-area: top;
}
.arm.bottom {
  grid-area: bottom;
}
.arm.left {
  grid-area: left;
}
.arm.right {
  grid-area: right;
}

.arm .btn {
  width: 28px;
  height: 28px;
  padding: 0;
  line-height: 1;
  font-size: 1rem;
}

.cross-center {
  grid-area: center;
  width: 32px;
  height: 32px;
  background: #f8f9fa;
  border-radius: 4px;
  border: 1px solid #dee2e6;
}
</style>
