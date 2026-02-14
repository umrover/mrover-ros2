<template>
  <div class="relative w-full h-full overflow-hidden" data-testid="pw-gimbal-controls">
    <div class="absolute top-0 left-0 right-0 flex justify-between items-center z-10">
      <h4 class="component-header">Gimbal</h4>
      <IndicatorDot :is-active="atTarget === null ? null : hasServoState && atTarget" />
    </div>

    <svg viewBox="16 16 168 168" class="gimbal-dial z-20">
      <g
        v-for="seg in arcSegments"
        :key="seg.id"
        :class="['segment', { disabled: btnDisabled, pending: pendingJoint === seg.joint }]"
        :data-testid="`pw-gimbal-${seg.joint}-btns`"
        role="button"
        @click="adjustGimbal(seg.joint, seg.delta)"
      >
        <path :d="seg.path" />
        <text :x="seg.lx" :y="seg.ly" class="seg-label">{{ seg.label }}</text>
      </g>

      <!-- Center readout -->
      <circle :cx="CX" :cy="CY" r="28" class="center-bg" />
      <text :x="CX" :y="CY - 16" class="center-axis">PITCH</text>
      <text :x="CX" :y="CY - 4" class="center-val">{{ jointDegrees('pitch') }}&deg;</text>
      <text :x="CX" :y="CY + 10" class="center-axis">YAW</text>
      <text :x="CX" :y="CY + 22" class="center-val">{{ jointDegrees('yaw') }}&deg;</text>
    </svg>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed } from 'vue'
import { chassisAPI } from '@/utils/chassisAPI'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import IndicatorDot from './IndicatorDot.vue'

type Joint = 'pitch' | 'yaw'

const DEG_TO_RAD = Math.PI / 180
const RAD_TO_DEG = 180 / Math.PI

// SVG geometry
const CX = 100
const CY = 100
const OUTER_R1 = 55
const OUTER_R2 = 80
const INNER_R1 = 28
const INNER_R2 = 51
const GAP_HALF = 2 // half the parallel gap width between segments

interface ArcSegment {
  id: string
  joint: Joint
  delta: number
  label: string
  path: string
  lx: number
  ly: number
}

// Compute where a parallel-offset gap edge intersects a circle.
// The gap is at angle gapDeg; the offset direction is toward segCenterDeg.
function gapEdgePoint(gapDeg: number, segCenterDeg: number, r: number): { x: number; y: number } {
  const gRad = gapDeg * DEG_TO_RAD
  const offsetSign = Math.sign(Math.sin((segCenterDeg - gapDeg) * DEG_TO_RAD))
  const t = Math.sqrt(r * r - GAP_HALF * GAP_HALF)
  return {
    x: CX + GAP_HALF * offsetSign * -Math.sin(gRad) + t * Math.cos(gRad),
    y: CY + GAP_HALF * offsetSign * Math.cos(gRad) + t * Math.sin(gRad),
  }
}

function segmentPath(gap1: number, gap2: number, center: number, r1: number, r2: number): string {
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

// Cardinal positions: top/bottom = pitch, left/right = yaw
// Angles are SVG convention: 0=right, 90=down, 180=left, 270=up
const POSITIONS = [
  { joint: 'pitch' as Joint, center: 270, gap1: 225, gap2: 315 },
  { joint: 'yaw' as Joint,   center: 0,   gap1: 315, gap2: 45 },
  { joint: 'pitch' as Joint, center: 90,  gap1: 45,  gap2: 135 },
  { joint: 'yaw' as Joint,   center: 180, gap1: 135, gap2: 225 },
]

function buildSegments(r1: number, r2: number, ring: string, magnitude: number): ArcSegment[] {
  return POSITIONS.map((pos) => {
    const delta = (pos.center === 90 || pos.center === 180) ? -magnitude : magnitude
    const midRad = pos.center * DEG_TO_RAD
    const midR = (r1 + r2) / 2
    return {
      id: `${ring}-${pos.joint}-${delta}`,
      joint: pos.joint,
      delta,
      label: delta > 0 ? `+${delta}` : `${delta}`,
      path: segmentPath(pos.gap1, pos.gap2, pos.center, r1, r2),
      lx: CX + midR * Math.cos(midRad),
      ly: CY + midR * Math.sin(midRad),
    }
  })
}

const arcSegments: ArcSegment[] = [
  ...buildSegments(OUTER_R1, OUTER_R2, 'outer', 10),
  ...buildSegments(INNER_R1, INNER_R2, 'inner', 1),
]

// Reactive state
const websocketStore = useWebsocketStore()
const isLoading = ref(false)
const atTarget = ref<boolean | null>(null)
const pendingJoint = ref<Joint | null>(null)

const gimbalJointState = computed((): ControllerStateMessage | null => {
  const msg = websocketStore.messages['chassis']
  if (!msg) return null
  const typedMsg = msg as ControllerStateMessage
  return typedMsg.type === 'gimbal_controller_state' ? typedMsg : null
})

const hasServoState = computed(() => gimbalJointState.value !== null)
const btnDisabled = computed(() => !hasServoState.value || isLoading.value)

function jointRadians(joint: Joint): number {
  const state = gimbalJointState.value
  if (!state?.names || !state.positions) return 0
  const idx = state.names.indexOf(joint)
  return idx >= 0 ? (state.positions[idx] ?? 0) : 0
}

function jointDegrees(joint: Joint): string {
  const deg = jointRadians(joint) * RAD_TO_DEG
  return Object.is(Math.round(deg), -0) ? '0' : deg.toFixed(0)
}

async function adjustGimbal(joint: Joint, deltaDegrees: number) {
  if (isLoading.value || !hasServoState.value) return

  const targetRadians = jointRadians(joint) + deltaDegrees * DEG_TO_RAD

  pendingJoint.value = joint
  isLoading.value = true

  try {
    const result = await chassisAPI.adjustGimbal(joint, targetRadians, true)
    atTarget.value = !!result.at_tgt
  } catch {
    atTarget.value = false
  } finally {
    isLoading.value = false
    pendingJoint.value = null
  }
}
</script>

<style scoped>
.gimbal-dial {
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

.segment.disabled {
  cursor: not-allowed;
}

.segment.disabled path {
  fill: var(--disabled-bg);
  stroke: var(--disabled-border);
}

.segment.disabled .seg-label {
  fill: var(--disabled-text);
}

.segment.pending path {
  fill: color-mix(in srgb, var(--text-muted) 20%, var(--card-bg));
}

.segment.pending .seg-label {
  fill: var(--text-muted);
}

.seg-label {
  font-size: 11px;
  font-weight: 600;
  fill: var(--control-primary);
  text-anchor: middle;
  dominant-baseline: central;
  pointer-events: none;
  user-select: none;
}

.center-bg {
  fill: var(--card-bg);
  stroke: var(--input-border);
  stroke-width: 1;
}

.center-axis {
  font-size: 7px;
  font-weight: 700;
  fill: var(--text-muted);
  text-anchor: middle;
  letter-spacing: 0.5px;
}

.center-val {
  font-size: 13px;
  font-weight: 700;
  fill: var(--text-primary, currentColor);
  text-anchor: middle;
}
</style>
