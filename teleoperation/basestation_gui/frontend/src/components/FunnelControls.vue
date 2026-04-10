<template>
  <div class="relative w-full h-full overflow-hidden" data-testid="pw-funnel-controls">
    <div class="absolute top-0 left-0 right-0 flex justify-between items-center z-10">
      <h4 class="component-header">Funnel</h4>
      <div class="flex items-center gap-1">
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm px-1.5 h-5!" :disabled="isLoading" @click="adjustOffset(-1)">-</button>
        <span class="text-[10px] text-muted select-none">{{ offsetDeg }}&deg;</span>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm px-1.5 h-5!" :disabled="isLoading" @click="adjustOffset(1)">+</button>
      </div>
    </div>

    <svg viewBox="10 10 180 180" class="funnel-dial">
      <defs>
        <path
          v-for="seg in segments"
          :key="`label-path-${seg.index}`"
          :id="`label-arc-${seg.index}`"
          :d="seg.labelArc"
          fill="none"
        />
      </defs>

      <g
        v-for="seg in segments"
        :key="seg.index"
        :class="['segment', { active: currentSite === seg.index, pending: pendingSite === seg.index, disabled: isLoading }]"
        :data-testid="`pw-funnel-site-${seg.index}`"
        role="button"
        @click="selectSite(seg.index)"
      >
        <path :d="seg.path" />
        <text class="seg-label">
          <textPath
            :href="`#label-arc-${seg.index}`"
            startOffset="50%"
            text-anchor="middle"
          >{{ seg.label }}</textPath>
        </text>
      </g>

      <circle :cx="CX" :cy="CY" r="32" class="center-bg" />

      <text :x="CX" :y="CY + 4" class="center-val">{{ currentDegrees }}&deg;</text>
    </svg>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { scienceAPI } from '@/utils/api'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'

const DEG_TO_RAD = Math.PI / 180
const RAD_TO_DEG = 180 / Math.PI

interface Site {
  label: string
  radians: number
}

const SITES: Site[] = [
  { label: 'Sample',   radians: 3.1415 },
  { label: 'Buret A',  radians: 2.0071 },
  { label: 'Griess A', radians: 1.1693 },
  { label: 'Trash',    radians: 0.0 },
  { label: 'Buret B',  radians: 5.1138 },
  { label: 'Griess B', radians: 4.2586 },
]

const CX = 100
const CY = 100
const INNER_R = 36
const OUTER_R = 78
const LABEL_R = 57
const SEGMENT_GAP_DEG = 4
const SLOT_SIZE = 360 / SITES.length
const SEGMENT_SPAN_DEG = SLOT_SIZE - SEGMENT_GAP_DEG

function polarToXY(angleDeg: number, r: number): { x: number; y: number } {
  const rad = (angleDeg - 90) * DEG_TO_RAD
  return { x: CX + r * Math.cos(rad), y: CY + r * Math.sin(rad) }
}

function arcPath(startDeg: number, endDeg: number, r1: number, r2: number): string {
  const p1 = polarToXY(startDeg, r1)
  const p2 = polarToXY(endDeg, r1)
  const p3 = polarToXY(endDeg, r2)
  const p4 = polarToXY(startDeg, r2)
  const f = (n: number) => n.toFixed(2)
  return [
    `M${f(p1.x)},${f(p1.y)}`,
    `A${r1},${r1} 0 0 1 ${f(p2.x)},${f(p2.y)}`,
    `L${f(p3.x)},${f(p3.y)}`,
    `A${r2},${r2} 0 0 0 ${f(p4.x)},${f(p4.y)}`,
    'Z',
  ].join(' ')
}

function labelArcPath(startDeg: number, endDeg: number, r: number, flip: boolean): string {
  const from = flip ? endDeg : startDeg
  const to = flip ? startDeg : endDeg
  const p1 = polarToXY(from, r)
  const p2 = polarToXY(to, r)
  const f = (n: number) => n.toFixed(2)
  return `M${f(p1.x)},${f(p1.y)} A${r},${r} 0 0 ${flip ? 0 : 1} ${f(p2.x)},${f(p2.y)}`
}

interface Segment {
  index: number
  label: string
  path: string
  labelArc: string
}

const segments: Segment[] = SITES.map((site, i) => {
  const startDeg = -(SLOT_SIZE / 2) + i * SLOT_SIZE + SEGMENT_GAP_DEG / 2
  const endDeg = startDeg + SEGMENT_SPAN_DEG
  const midDeg = (startDeg + endDeg) / 2
  const flip = midDeg > 120 && midDeg < 240
  return {
    index: i,
    label: site.label,
    path: arcPath(startDeg, endDeg, INNER_R, OUTER_R),
    labelArc: labelArcPath(startDeg, endDeg, LABEL_R, flip),
  }
})

const websocketStore = useWebsocketStore()

const funnelState = computed((): ControllerStateMessage | null => {
  const msg = websocketStore.messages['science']
  if (!msg) return null
  const typedMsg = msg as ControllerStateMessage
  return typedMsg.type === 'sp_controller_state' ? typedMsg : null
})

const currentDegrees = computed((): string => {
  const state = funnelState.value
  if (!state?.names || !state.positions) return '--'
  const idx = state.names.indexOf('funnel')
  if (idx < 0) return '--'
  const deg = (state.positions[idx] ?? 0) * RAD_TO_DEG
  return Object.is(Math.round(deg), -0) ? '0' : deg.toFixed(0)
})

const currentSite = ref(0)
const pendingSite = ref<number | null>(null)
const isLoading = ref(false)
const offsetDeg = ref(Number(localStorage.getItem('funnel-offset-deg')) || 0)

async function adjustOffset(delta: number) {
  if (isLoading.value) return
  offsetDeg.value += delta
  try {
    await sendPosition(currentSite.value)
    localStorage.setItem('funnel-offset-deg', String(offsetDeg.value))
  } catch {
    offsetDeg.value -= delta
  }
}

async function sendPosition(index: number) {
  if (isLoading.value) return
  const site = SITES[index]
  if (!site) return

  const previousSite = currentSite.value
  currentSite.value = index
  pendingSite.value = index
  isLoading.value = true

  try {
    const radians = ((site.radians + offsetDeg.value * DEG_TO_RAD) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI)
    await scienceAPI.setGearDiffPosition(radians, false)
  } catch {
    currentSite.value = previousSite
  } finally {
    isLoading.value = false
    pendingSite.value = null
  }
}

function selectSite(index: number) {
  if (index === currentSite.value || isLoading.value) return
  sendPosition(index)
}

const { buttons } = useGamepadPolling({ controllerIdFilter: 'Microsoft', hz: 10 })
let prevLeft = false
let prevRight = false

watch(buttons, (btns) => {
  const left = (btns[14] ?? 0) > 0.5
  const right = (btns[15] ?? 0) > 0.5
  if (left && !prevLeft) adjustOffset(-1)
  if (right && !prevRight) adjustOffset(1)
  prevLeft = left
  prevRight = right
})
</script>

<style scoped>
.funnel-dial {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
}

.segment {
  cursor: pointer;
}

.segment path {
  fill: color-mix(in srgb, var(--control-primary) 8%, var(--card-bg));
  stroke: var(--input-border);
  stroke-width: 1;
  transition: fill 0.15s ease;
}

.segment:hover:not(.disabled) path {
  fill: var(--control-primary);
}

.segment:hover:not(.disabled) .seg-label {
  fill: #fff;
}

.segment.active path {
  fill: var(--control-primary);
}

.segment.active .seg-label {
  fill: #fff;
}

.segment.disabled {
  cursor: not-allowed;
}

.segment.pending path {
  fill: color-mix(in srgb, var(--text-muted) 20%, var(--card-bg));
}

.seg-label {
  font-size: 10px;
  font-weight: 600;
  fill: var(--control-primary);
  pointer-events: none;
  user-select: none;
}

.center-bg {
  fill: var(--card-bg);
  stroke: var(--input-border);
  stroke-width: 1;
}

.center-val {
  font-size: 13px;
  font-weight: 700;
  fill: var(--text-primary, currentColor);
  text-anchor: middle;
}
</style>
