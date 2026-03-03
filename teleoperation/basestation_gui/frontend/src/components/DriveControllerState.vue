<template>
  <div class="d-flex flex-column border border-2 p-2 rounded mw-100" style="flex: 1 0 auto; min-width: 0;">
    <h4 class="m-0 mb-1">Drive</h4>
    <svg viewBox="0 0 280 100" class="rover-svg" preserveAspectRatio="xMidYMid meet">
      <rect x="110" y="10" width="60" height="80" rx="4" fill="#e9ecef" stroke="#495057" stroke-width="1.5" />

      <g v-for="(w, idx) in wheels" :key="w.id">
        <line :x1="w.lineX1" :y1="w.y + 6" :x2="w.lineX2" :y2="w.y + 6" stroke="#495057" stroke-width="1" />
        <rect
          :x="w.x" :y="w.y" width="16" height="12" rx="2"
          :fill="stateColor(states[idx])" stroke="#495057" stroke-width="1"
        />
        <text
          :x="w.labelX" :y="w.y + 5"
          :text-anchor="w.left ? 'end' : 'start'"
          font-size="5.5" font-weight="bold" fill="#212529"
        >
          {{ states[idx] || 'OFFLINE' }}
        </text>
        <text
          :x="w.labelX" :y="w.y + 12"
          :text-anchor="w.left ? 'end' : 'start'"
          font-size="5" fill="#495057"
        >
          {{ formatVelocity(velocities[idx]) }} m/s
        </text>
      </g>
    </svg>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage } from '@/types/websocket'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const states = ref<string[]>([])
const velocities = ref<number[]>([])

const leftState = ref<ControllerStateMessage | null>(null)
const rightState = ref<ControllerStateMessage | null>(null)

function combineLeftRight() {
  const l = leftState.value || { states: [], velocities: [] }
  const r = rightState.value || { states: [], velocities: [] }

  states.value = [...l.states, ...r.states]
  velocities.value = [...l.velocities, ...r.velocities]
}

const driveMessage = computed(() => messages.value['drive'])

watch(driveMessage, (msg) => {
  if (!msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'drive_left_state') {
    leftState.value = typed
    combineLeftRight()
  } else if (typed.type === 'drive_right_state') {
    rightState.value = typed
    combineLeftRight()
  }
})

const wheels = [
  { id: 'fl', left: true,  x: 88,  y: 10, lineX1: 104, lineX2: 110, labelX: 86 },
  { id: 'ml', left: true,  x: 88,  y: 40, lineX1: 104, lineX2: 110, labelX: 86 },
  { id: 'rl', left: true,  x: 88,  y: 70, lineX1: 104, lineX2: 110, labelX: 86 },
  { id: 'fr', left: false, x: 176, y: 10, lineX1: 170, lineX2: 176, labelX: 194 },
  { id: 'mr', left: false, x: 176, y: 40, lineX1: 170, lineX2: 176, labelX: 194 },
  { id: 'rr', left: false, x: 176, y: 70, lineX1: 170, lineX2: 176, labelX: 194 },
]

function stateColor(state: string | undefined): string {
  if (!state || state === 'OFFLINE') return '#f8d7da'
  if (state === 'ARMED') return '#d1e7dd'
  return '#fff3cd'
}

function formatVelocity(v: unknown): string {
  if (typeof v === 'number' && Number.isFinite(v)) return v.toFixed(2)
  return '0.00'
}
</script>

<style scoped>
.rover-svg {
  max-width: 100%;
  max-height: 100%;
}
</style>
