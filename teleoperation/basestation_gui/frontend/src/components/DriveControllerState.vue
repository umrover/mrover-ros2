<template>
  <div class="cmd-panel flex flex-col h-full">
    <div class="cmd-panel-header">
      <h4>Drive</h4>
    </div>
    <div class="grid grid-cols-2 gap-1 flex-1">
      <div
        v-for="w in wheels"
        :key="w.id"
        class="rounded p-1.5 text-center border"
        :class="stateClass(states[w.idx])"
      >
        <div class="text-xs font-bold tracking-wide" style="color: var(--text-primary)">{{ w.label }}</div>
        <div class="text-xs font-semibold">{{ states[w.idx] || 'OFFLINE' }}</div>
        <div class="text-xs" style="color: var(--text-muted)">{{ formatNumber(velocities[w.idx]) }} m/s</div>
        <div class="text-xs" style="color: var(--text-muted)">{{ formatNumber(currents[w.idx]) }} A</div>
        <div v-if="errors[w.idx]" class="text-xs text-cmd-danger">{{ errors[w.idx] }}</div>
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
const currents = ref<number[]>([])
const errors = ref<string[]>([])

const leftState = ref<ControllerStateMessage | null>(null)
const rightState = ref<ControllerStateMessage | null>(null)

function combineLeftRight() {
  const l = leftState.value || { states: [], velocities: [], currents: [], errors: [] }
  const r = rightState.value || { states: [], velocities: [], currents: [], errors: [] }

  states.value = [...l.states, ...r.states]
  velocities.value = [...l.velocities, ...r.velocities]
  currents.value = [...l.currents, ...r.currents]
  errors.value = [...l.errors, ...r.errors]
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

function stateClass(state: string | undefined): string {
  if (!state || state === 'OFFLINE') return 'bg-cmd-danger-subtle border-theme'
  if (state === 'ARMED') return 'bg-cmd-success-subtle border-theme'
  return 'border-theme'
}

function formatNumber(v: unknown): string {
  if (typeof v === 'number' && Number.isFinite(v)) return v.toFixed(2)
  return '0.00'
}
</script>
