<template>
  <div class="d-flex flex-column border border-2 p-2 rounded mw-100" style="flex: 1 0 auto; min-width: 0;">
    <h4 class="m-0 mb-1">Drive</h4>
    <div class="wheel-grid">
      <div v-for="w in wheels" :key="w.id" class="wheel-cell rounded p-1 text-center">
        <div class="fw-bold small">{{ w.label }}</div>
        <div class="small" :style="{ color: stateColor(states[w.idx]) }">
          {{ states[w.idx] || 'OFFLINE' }}
        </div>
        <div class="text-muted small">{{ formatVelocity(velocities[w.idx]) }} m/s</div>
      </div>
    </div>
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
  { id: 'fl', label: 'FL', idx: 0 },
  { id: 'fr', label: 'FR', idx: 3 },
  { id: 'ml', label: 'ML', idx: 1 },
  { id: 'mr', label: 'MR', idx: 4 },
  { id: 'rl', label: 'RL', idx: 2 },
  { id: 'rr', label: 'RR', idx: 5 },
]

function stateColor(state: string | undefined): string {
  if (!state || state === 'OFFLINE') return '#dc3545'
  if (state === 'ARMED') return '#198754'
  return '#ffc107'
}

function formatVelocity(v: unknown): string {
  if (typeof v === 'number' && Number.isFinite(v)) return v.toFixed(2)
  return '0.00'
}
</script>

<style scoped>
.wheel-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 4px;
}

.wheel-cell {
  background: #f8f9fa;
  border: 1px solid #dee2e6;
}
</style>
