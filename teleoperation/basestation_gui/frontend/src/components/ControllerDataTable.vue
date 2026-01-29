<template>
  <div class='d-flex flex-column border border-2 p-2 rounded mw-100' style="flex: 1 0 auto; min-width: 0;">
    <div class="d-flex align-items-center justify-content-between mb-2">
      <h4 class="m-0">{{ header }}</h4>
      <div class="d-flex gap-2">
        <button
          type="button"
          class="btn btn-sm"
          :class="showStatus ? 'btn-success' : 'btn-danger'"
          @mousedown.prevent
          @click="showStatus = !showStatus"
        >Status</button>
        <button
          type="button"
          class="btn btn-sm"
          :class="showValues ? 'btn-success' : 'btn-danger'"
          @mousedown.prevent
          @click="showValues = !showValues"
        >Values</button>
      </div>
    </div>

    <!-- Replaced the previous table with an interactive SVG rover view -->
    <div class="d-flex gap-2">
      <div class="flex-grow-1 d-flex align-items-center justify-content-center p-2" style="min-width:0;">
        <svg viewBox="0 0 200 110" class="rover-svg" preserveAspectRatio="xMidYMid meet" role="img" aria-label="Rover diagram">
          <!-- body -->
          <rect x="30" y="25" width="140" height="60" rx="6" fill="#e9ecef" stroke="#adb5bd" />
          <!-- visual wheel placeholders; clickable -->
          <g v-for="(w, idx) in wheelPositions" :key="w.id" :transform="`translate(${w.x}, ${w.y})`" class="wheel-group" @click="selectedIndex = idx" :title="`Select ${w.label}`" style="cursor: pointer;">
            <circle :r="8" :fill="selectedIndex === idx ? '#0d6efd' : '#fff'" stroke="#495057" stroke-width="1.5" />
            <text x="0" y="3" font-size="6" text-anchor="middle" fill="#495057">{{ w.short }}</text>
          </g>

          <!-- labels for front/back -->
          <text x="100" y="18" font-size="7" text-anchor="middle" fill="#495057">{{ header }}</text>
        </svg>
      </div>

      <div class="details-panel p-2 border-start" style="width: 220px; min-width: 140px;">
        <div class="mb-1 small text-muted">Selected Part</div>
        <div class="fw-bold mb-2">{{ selectedData.name ?? 'None' }}</div>

        <div v-if="showStatus" class="small mb-2">
          <div><span class="text-muted">State:</span> <span>{{ selectedData.state }}</span></div>
          <div><span class="text-muted">Error:</span> <span>{{ selectedData.error }}</span></div>
          <div><span class="text-muted">Limit Hit:</span> <span>{{ selectedData.limitHit ? 'Yes' : 'No' }}</span></div>
        </div>

        <div v-if="showValues" class="small">
          <div><span class="text-muted">Position:</span> <span>{{ formatNumber(selectedData.position) }}</span></div>
          <div><span class="text-muted">Velocity:</span> <span>{{ formatNumber(selectedData.velocity) }}</span></div>
          <div><span class="text-muted">Current:</span> <span>{{ formatNumber(selectedData.current) }}</span></div>
        </div>

        <div v-if="selectedIndex === -1" class="small text-muted mt-2">Click a wheel to view data.</div>
      </div>
    </div>
  </div>
</template>

<script lang='ts' setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage } from '@/types/websocket'

const props = defineProps<{
  header: string
  mode: 'drive' | 'arm' | 'sp'
}>()

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const statusKey = `controllerDataTable_${props.mode}_showStatus`
const valuesKey = `controllerDataTable_${props.mode}_showValues`

const showStatus = ref(localStorage.getItem(statusKey) === 'true')
const showValues = ref(localStorage.getItem(valuesKey) === 'true')

watch(showStatus, (val) => localStorage.setItem(statusKey, String(val)))
watch(showValues, (val) => localStorage.setItem(valuesKey, String(val)))

const names = ref<string[]>([])
const states = ref<string[]>([])
const errors = ref<string[]>([])
const limitHits = ref<boolean[]>([])
const positions = ref<number[]>([])
const velocities = ref<number[]>([])
const currents = ref<number[]>([])

const leftState = ref<ControllerStateMessage | null>(null)
const rightState = ref<ControllerStateMessage | null>(null)

function updateFromMessage(msg: ControllerStateMessage) {
  names.value = msg.names
  states.value = msg.states
  errors.value = msg.errors
  limitHits.value = msg.limits_hit
  positions.value = msg.positions
  velocities.value = msg.velocities
  currents.value = msg.currents
}

function combineLeftRight() {
  const left = leftState.value
  const right = rightState.value
  if (!left && !right) return

  const l = left || { names: [], states: [], errors: [], limits_hit: [], positions: [], velocities: [], currents: [] }
  const r = right || { names: [], states: [], errors: [], limits_hit: [], positions: [], velocities: [], currents: [] }

  names.value = [...l.names, ...r.names]
  states.value = [...l.states, ...r.states]
  errors.value = [...l.errors, ...r.errors]
  limitHits.value = [...l.limits_hit, ...r.limits_hit]
  positions.value = [...l.positions, ...r.positions]
  velocities.value = [...l.velocities, ...r.velocities]
  currents.value = [...l.currents, ...r.currents]
}

const driveMessage = computed(() => messages.value['drive'])
const armMessage = computed(() => messages.value['arm'])
const scienceMessage = computed(() => messages.value['science'])

watch(driveMessage, (msg) => {
  if (props.mode !== 'drive' || !msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'drive_left_state') {
    leftState.value = typed
    combineLeftRight()
  } else if (typed.type === 'drive_right_state') {
    rightState.value = typed
    combineLeftRight()
  }
})

watch(armMessage, (msg) => {
  if (props.mode !== 'arm' || !msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'arm_state') {
    updateFromMessage(typed)
  }
})

watch(scienceMessage, (msg) => {
  if (props.mode !== 'sp' || !msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'sp_controller_state') {
    updateFromMessage(typed)
  }
})

// --- New interactive view state ---

// Selected wheel/index (-1 means none)
const selectedIndex = ref<number>(-1)

// Wheel visual positions and labels in SVG coordinates
const wheelPositions = [
  { id: 'front_left', label: 'Front Left Wheel', short: 'FL', x: 50, y: 30 },
  { id: 'front_right', label: 'Front Right Wheel', short: 'FR', x: 150, y: 30 },
  { id: 'mid_left', label: 'Middle Left Wheel', short: 'ML', x: 50, y: 55 },
  { id: 'mid_right', label: 'Middle Right Wheel', short: 'MR', x: 150, y: 55 },
  { id: 'rear_left', label: 'Rear Left Wheel', short: 'RL', x: 50, y: 80 },
  { id: 'rear_right', label: 'Rear Right Wheel', short: 'RR', x: 150, y: 80 },
]

// Helper to safely format numbers
function formatNumber(v: unknown) {
  if (typeof v === 'number' && Number.isFinite(v)) return v.toFixed(2)
  return '—'
}

const selectedData = computed(() => {
  const idx = selectedIndex.value
  if (idx < 0 || idx >= names.value.length) {
    return {
      name: null,
      state: '—',
      error: '—',
      limitHit: false,
      position: null,
      velocity: null,
      current: null,
    }
  }
  return {
    name: names.value[idx] ?? `#${idx}`,
    state: states.value[idx] ?? '—',
    error: errors.value[idx] ?? '—',
    limitHit: !!limitHits.value[idx],
    position: positions.value[idx] ?? null,
    velocity: velocities.value[idx] ?? null,
    current: currents.value[idx] ?? null,
  }
})
</script>

<style scoped>
.sticky-col {
  position: sticky;
  left: 0;
  z-index: 1;
}

.compact-table {
  table-layout: auto;
}

.compact-table th,
.compact-table td {
  white-space: nowrap;
  width: 1%;
}

/* New styles for interactive rover view */
.rover-svg {
  width: 100%;
  height: 220px;
  max-height: 260px;
  background: linear-gradient(180deg, rgba(255,255,255,0.6), rgba(250,250,250,0.9));
  border-radius: 6px;
  box-sizing: border-box;
}

.wheel-group:hover circle {
  stroke-width: 2;
  transform: scale(1.08);
  transform-origin: center;
}

/* small details panel */
.details-panel {
  background: transparent;
}
</style>
