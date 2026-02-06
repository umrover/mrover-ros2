<template>
  <div class='d-flex flex-column border border-2 p-2 rounded mw-100' style="flex: 1 0 auto; min-width: 0;">
    <div class="d-flex align-items-center justify-content-between mb-2">
      <h4 class="m-0">{{ header }}</h4>
      <div class="d-flex gap-2" v-if="mode !== 'arm'">
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

    <div class="d-flex gap-2">
      <div class="flex-grow-1 d-flex align-items-center justify-content-center" style="min-width:0;">
        <svg viewBox="10 5 220 95" class="rover-svg" preserveAspectRatio="xMidYMid meet" role="img" aria-label="Rover diagram">
          <image
            :href="roverImageUrl"
            x="50"
            y="25"
            width="140"
            height="60"
            preserveAspectRatio="xMidYMid meet"
          />

          <g v-for="(w, idx) in wheelPositions" :key="w.id" :transform="`translate(${w.x}, ${w.y})`" class="wheel-group" @click="selectedIndex = idx" style="cursor: pointer;">
            <g class="live-data">
              <text 
                :x="w.x < 120 ? -14 : 14" 
                y="-2" 
                :text-anchor="w.x < 120 ? 'end' : 'start'" 
                font-size="7" 
                font-weight="bold" 
                fill="#212529"
              >
                {{ states[idx] || 'OFFLINE' }}
              </text>
              <text 
                :x="w.x < 120 ? -14 : 14" 
                y="7" 
                :text-anchor="w.x < 120 ? 'end' : 'start'" 
                font-size="7" 
                fill="#6c757d"
              >
                {{ formatNumber(velocities[idx]) }} m/s
              </text>
            </g>

            <circle :r="10" :fill="selectedIndex === idx ? '#0d6efd' : '#fff'" stroke="#495057" stroke-width="1.5" />
            <text x="0" y="3" font-size="8" text-anchor="middle" :fill="selectedIndex === idx ? '#fff' : '#495057'" font-weight="bold">
              {{ w.short }}
            </text>
          </g>

          <text x="120" y="16" font-size="10" text-anchor="middle" fill="#495057" font-weight="bold">{{ header }}</text>
        </svg>
      </div>

      <div v-if="mode !== 'arm'" class="details-panel p-2 border-start">
  <div class="mb-1 small text-muted">Selected Part</div>
  <div class="fw-bold mb-2">{{ selectedData.name ?? 'None' }}</div>

  <div v-if="showStatus" class="small mb-2">
    <div v-if="mode !== 'drive'"><span class="text-muted">State:</span> <span>{{ selectedData.state }}</span></div>
    
    <div><span class="text-muted">Error:</span> <span>{{ selectedData.error }}</span></div>
    <div><span class="text-muted">Limit Hit:</span> <span>{{ selectedData.limitHit ? 'Yes' : 'No' }}</span></div>
  </div>

  <div v-if="showValues" class="small">
    <div><span class="text-muted">Position:</span> <span>{{ formatNumber(selectedData.position) }}</span></div>
    
    <div v-if="mode !== 'drive'"><span class="text-muted">Velocity:</span> <span>{{ formatNumber(selectedData.velocity) }}</span></div>
    
    <div><span class="text-muted">Current:</span> <span>{{ formatNumber(selectedData.current) }}</span></div>
  </div>

  <div v-if="selectedIndex === -1" class="small text-muted mt-2">Click a wheel to view detailed logs.</div>
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
  const l = leftState.value || { names: [], states: [], errors: [], limits_hit: [], positions: [], velocities: [], currents: [] }
  const r = rightState.value || { names: [], states: [], errors: [], limits_hit: [], positions: [], velocities: [], currents: [] }

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
  updateFromMessage(msg as ControllerStateMessage)
})

watch(scienceMessage, (msg) => {
  if (props.mode !== 'sp' || !msg) return
  updateFromMessage(msg as ControllerStateMessage)
})

const selectedIndex = ref<number>(-1)

// Adjusted X coordinates to make room for labels within the 240px viewBox
const wheelPositions = [
  { id: 'front_left', label: 'Front Left', short: 'FL', x: 60, y: 30 },
  { id: 'mid_left', label: 'Middle Left', short: 'ML', x: 60, y: 55 },
  { id: 'rear_left', label: 'Rear Left', short: 'RL', x: 60, y: 80 },
  { id: 'front_right', label: 'Front Right', short: 'FR', x: 180, y: 30 },
  { id: 'mid_right', label: 'Middle Right', short: 'MR', x: 180, y: 55 },
  { id: 'rear_right', label: 'Rear Right', short: 'RR', x: 180, y: 80 },
]

function formatNumber(v: unknown) {
  if (typeof v === 'number' && Number.isFinite(v)) return v.toFixed(2)
  return '0.00'
}

const selectedData = computed(() => {
  const idx = selectedIndex.value
  if (idx < 0 || idx >= names.value.length) {
    return { name: null, state: '—', error: '—', limitHit: false, position: null, velocity: null, current: null }
  }
  return {
    name: names.value[idx],
    state: states.value[idx],
    error: errors.value[idx],
    limitHit: !!limitHits.value[idx],
    position: positions.value[idx],
    velocity: velocities.value[idx],
    current: currents.value[idx],
  }
})

// Use the DM_Rover_image.svg in the public root directory
const roverImageUrl = `${import.meta.env.BASE_URL}roverForArmDrivePNG.png`
</script>

<style scoped>
.rover-svg {
  width: 100%;
  height: auto;
  max-height: 150px;
  aspect-ratio: 220 / 95;
  background: #f8f9fa;
  border-radius: 6px;
}

.wheel-group:hover circle {
  stroke: #0d6efd;
  transition: all 0.2s ease;
}

.details-panel {
  background: white;
  width: 150px;
  min-width: 120px;
}

.live-data text {
  pointer-events: none; /* Prevents text from blocking clicks on the wheel */
  user-select: none;
}
</style>