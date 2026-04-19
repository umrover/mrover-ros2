<template>
  <div class="relative w-full h-full overflow-hidden" data-testid="pw-funnel-controls">
    <div class="absolute top-0 left-0 right-0 flex justify-between items-center z-10">
      <h4 class="component-header">Funnel</h4>
      <div class="flex items-center">
        <i
          v-if="!hasState"
          class="bi bi-info-circle mr-1 text-sm text-muted"
          title="Changes not allowed until funnel position received"
        ></i>
        <IndicatorDot :is-active="hasState" class="mr-2" />
        <div class="flex items-center gap-0.5 text-[11px]">
          <button class="micro-btn" :disabled="!hasState || isLoading" @click="adjustOffset(-1)">-</button>
          <span class="text-muted select-none tabular-nums min-w-[2.5ch] text-center">{{ offsetDeg }}&deg;</span>
          <button class="micro-btn" :disabled="!hasState || isLoading" @click="adjustOffset(1)">+</button>
        </div>
      </div>
    </div>

    <svg viewBox="0 0 200 200" class="funnel-dial">
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
        :class="['segment', { active: currentSite === seg.index, pending: pendingSite === seg.index, disabled: !hasState || isLoading }]"
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

      <text :x="CX" :y="CY + 6" class="center-val">{{ currentDegrees }}&deg;</text>
    </svg>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { scienceAPI } from '@/utils/api'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import IndicatorDot from './IndicatorDot.vue'

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
const INNER_R = 40
const OUTER_R = 100
const LABEL_R = 80
const LABEL_R_FLIP = 90
const GAP_HALF = 3
const SLOT_SIZE = 360 / SITES.length

function polarToXY(angleDeg: number, r: number): { x: number; y: number } {
  const rad = (angleDeg - 90) * DEG_TO_RAD
  return { x: CX + r * Math.cos(rad), y: CY + r * Math.sin(rad) }
}

function gapEdgePoint(gapDeg: number, segCenterDeg: number, r: number): { x: number; y: number } {
  const gRad = (gapDeg - 90) * DEG_TO_RAD
  const offsetSign = Math.sign(Math.sin((segCenterDeg - gapDeg) * DEG_TO_RAD))
  const t = Math.sqrt(r * r - GAP_HALF * GAP_HALF)
  return {
    x: CX + GAP_HALF * offsetSign * -Math.sin(gRad) + t * Math.cos(gRad),
    y: CY + GAP_HALF * offsetSign * Math.cos(gRad) + t * Math.sin(gRad),
  }
}

function arcPath(gap1: number, gap2: number, center: number, r1: number, r2: number): string {
  const p1 = gapEdgePoint(gap1, center, r1)
  const p2 = gapEdgePoint(gap2, center, r1)
  const p3 = gapEdgePoint(gap2, center, r2)
  const p4 = gapEdgePoint(gap1, center, r2)
  const f = (n: number) => n.toFixed(2)
  return [
    `M${f(p1.x)},${f(p1.y)}`,
    `A${r1},${r1} 0 0 1 ${f(p2.x)},${f(p2.y)}`,
    `L${f(p3.x)},${f(p3.y)}`,
    `A${r2},${r2} 0 0 0 ${f(p4.x)},${f(p4.y)}`,
    'Z',
  ].join(' ')
}

function labelArcPath(gap1: number, gap2: number, r: number, flip: boolean): string {
  const from = flip ? gap2 : gap1
  const to = flip ? gap1 : gap2
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
  const centerDeg = i * SLOT_SIZE
  const gap1 = centerDeg - SLOT_SIZE / 2
  const gap2 = centerDeg + SLOT_SIZE / 2
  const flip = centerDeg > 90 && centerDeg < 270
  return {
    index: i,
    label: site.label,
    path: arcPath(gap1, gap2, centerDeg, INNER_R, OUTER_R),
    labelArc: labelArcPath(gap1, gap2, flip ? LABEL_R_FLIP : LABEL_R, flip),
  }
})

const websocketStore = useWebsocketStore()

const funnelState = computed((): ControllerStateMessage | null => {
  const msg = websocketStore.messages['science']
  if (!msg) return null
  const typedMsg = msg as ControllerStateMessage
  return typedMsg.type === 'sp_controller_state' ? typedMsg : null
})

const hasState = computed(() => funnelState.value !== null)

const currentDegrees = computed((): string => {
  const state = funnelState.value
  if (!state?.names || !state.positions) return '---'
  const idx = state.names.indexOf('funnel')
  if (idx < 0) return '---'
  const deg = (state.positions[idx] ?? 0) * RAD_TO_DEG
  return Object.is(Math.round(deg), -0) ? '0' : deg.toFixed(0)
})

const currentSite = ref(-1)
const pendingSite = ref<number | null>(null)
const isLoading = ref(false)
const offsetDeg = ref(Number(localStorage.getItem('funnel-offset-deg')) || 0)

watch([funnelState, offsetDeg], ([state, offset]) => {
  if (!state || isLoading.value) return
  const idx = state.names.indexOf('funnel')
  if (idx < 0) return

  const pos = state.positions[idx]
  let closestIdx = -1
  let minDiff = Infinity

  for (let i = 0; i < SITES.length; i++) {
    const siteRadWithOffset = SITES[i].radians + (offset as number) * DEG_TO_RAD
    const diff = Math.abs(((siteRadWithOffset - pos + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI)
    if (diff < minDiff) {
      minDiff = diff
      closestIdx = i
    }
  }

  if (closestIdx !== -1 && minDiff < 0.2 && currentSite.value !== closestIdx) {
    currentSite.value = closestIdx
  }
}, { immediate: true })

async function adjustOffset(delta: number) {
  if (isLoading.value || !hasState.value) return
  offsetDeg.value += delta
  try {
    await sendPosition(currentSite.value)
    localStorage.setItem('funnel-offset-deg', String(offsetDeg.value))
  } catch {
    offsetDeg.value -= delta
  }
}

async function sendPosition(index: number) {
  if (isLoading.value || !hasState.value || index === -1) return
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
  if (index === currentSite.value || isLoading.value || !hasState.value) return
  sendPosition(index)
}

const { buttons, connected } = useGamepadPolling({ controllerIdFilter: 'Microsoft', hz: 10 })
let prevLeft: boolean | undefined = undefined
let prevRight: boolean | undefined = undefined

watch(buttons, (btns) => {
  if (!connected.value || !hasState.value) return

  const left = (btns[14] ?? 0) > 0.5
  const right = (btns[15] ?? 0) > 0.5

  if (prevLeft !== undefined && prevRight !== undefined) {
    if (left && !prevLeft) adjustOffset(-1)
    if (right && !prevRight) adjustOffset(1)
  }

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

.segment.disabled path {
  fill: var(--disabled-bg, #f3f4f6);
  stroke: var(--disabled-border, #e5e7eb);
}

.segment.disabled .seg-label {
  fill: var(--disabled-text, #9ca3af);
}

.segment.pending path {
  fill: color-mix(in srgb, var(--text-muted) 20%, var(--card-bg));
}

.seg-label {
  font-size: 16px;
  font-weight: 600;
  fill: var(--control-primary);
  pointer-events: none;
  user-select: none;
}

.micro-btn {
  width: 18px;
  height: 18px;
  line-height: 1;
  border-radius: 4px;
  border: 1px solid var(--input-border);
  background: transparent;
  color: var(--control-primary);
  font-weight: 700;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 0;
  transition: background 0.15s ease;
}

.micro-btn:hover:not(:disabled) {
  background: var(--control-primary);
  color: #fff;
}

.micro-btn:disabled {
  opacity: 0.4;
  cursor: not-allowed;
}

.center-val {
  font-size: 20px;
  font-weight: 700;
  fill: var(--text-primary, currentColor);
  text-anchor: middle;
}
</style>
