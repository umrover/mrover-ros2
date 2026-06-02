<template>
  <div class="relative w-full h-full" data-testid="pw-funnel-controls">
    <div class="absolute top-0 left-0 right-0 flex justify-between items-center z-10">
      <h4 class="component-header">Funnel</h4>
      <div class="flex items-center gap-2">
        <i
          v-if="!hasState"
          class="bi bi-info-circle text-sm text-muted"
          title="Changes not allowed until funnel position received"
        ></i>
        <button
          class="tare-btn"
          :disabled="!hasState || offsetDeg === 0"
          title="Reset offset to 0"
          @click="tare"
        >Tare</button>
        <IndicatorDot :is-active="hasState" class="mr-2" />
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
        <path
          v-for="seg in segments"
          :key="`target-path-${seg.index}`"
          :id="`target-arc-${seg.index}`"
          :d="seg.targetArc"
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
        <text class="seg-target">
          <textPath
            :href="`#target-arc-${seg.index}`"
            startOffset="50%"
            text-anchor="middle"
          >{{ siteTargets[seg.index] }}&deg;</textPath>
        </text>
      </g>

      <g
        v-for="blade in leftBlades"
        :key="`lb${blade.delta}`"
        :class="['blade', { disabled: !hasState || isLoading }]"
        role="button"
        @click="adjustOffset(blade.delta)"
      >
        <path :d="blade.path" />
        <text
          :x="blade.tx" :y="blade.ty"
          class="blade-label"
          dominant-baseline="middle"
          :transform="`rotate(${blade.rot},${blade.tx},${blade.ty})`"
        >{{ blade.label }}</text>
      </g>

      <g
        v-for="blade in rightBlades"
        :key="`rb${blade.delta}`"
        :class="['blade', { disabled: !hasState || isLoading }]"
        role="button"
        @click="adjustOffset(blade.delta)"
      >
        <path :d="blade.path" />
        <text
          :x="blade.tx" :y="blade.ty"
          class="blade-label"
          dominant-baseline="middle"
          :transform="`rotate(${blade.rot},${blade.tx},${blade.ty})`"
        >{{ blade.label }}</text>
      </g>

      <text :x="CX" :y="CY + 3" class="center-val">{{ currentDegrees }}&deg;</text>
      <text :x="CX" :y="CY + 17" class="offset-readout">{{ offsetDeg >= 0 ? '+' : '' }}{{ offsetDeg }}&deg;</text>
    </svg>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { scienceAPI } from '@/utils/api'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import IndicatorDot from './IndicatorDot.vue'
import {
  CX,
  CY,
  RAD_TO_DEG,
  SITES,
  segments,
  closestSiteIndex,
  siteTargetDeg,
  siteTargetRad,
  leftBlades,
  rightBlades,
} from './funnelGeometry'

const OFFSET_STORAGE_KEY = 'funnel-offset-deg'

const funnelState = ref<ControllerStateMessage | null>(null)
const currentSite = ref(-1)
const pendingSite = ref<number | null>(null)
const isLoading = ref(false)
const offsetDeg = ref(Number(localStorage.getItem(OFFSET_STORAGE_KEY)) || 0)

const hasState = computed(() => funnelState.value !== null)

const currentDegrees = computed((): string => {
  const state = funnelState.value
  if (!state?.names || !state.positions) return '---'
  const idx = state.names.indexOf('funnel')
  if (idx < 0) return '---'
  const deg = ((state.positions[idx] ?? 0) * RAD_TO_DEG) % 360
  return Object.is(Math.round(deg), -0) ? '0' : deg.toFixed(0)
})

const siteTargets = computed(() =>
  SITES.map((_, i) => siteTargetDeg(i, offsetDeg.value))
)

watch([funnelState, offsetDeg], ([state, offset]) => {
  if (!state || isLoading.value) return
  const idx = state.names.indexOf('funnel')
  if (idx < 0) return
  const match = closestSiteIndex(state.positions[idx], offset as number)
  if (match !== -1 && currentSite.value !== match) {
    currentSite.value = match
  }
}, { immediate: true })

async function sendPosition(index: number) {
  if (isLoading.value || !hasState.value || index === -1) return
  if (!SITES[index]) return

  const previousSite = currentSite.value
  currentSite.value = index
  pendingSite.value = index
  isLoading.value = true

  try {
    await scienceAPI.setFunnelPosition(siteTargetRad(index, offsetDeg.value), false)
    localStorage.setItem(OFFSET_STORAGE_KEY, String(offsetDeg.value))
  } catch (err) {
    currentSite.value = previousSite
    throw err
  } finally {
    isLoading.value = false
    pendingSite.value = null
  }
}

async function adjustOffset(delta: number) {
  if (isLoading.value || !hasState.value) return
  offsetDeg.value += delta
  if (currentSite.value === -1) {
    localStorage.setItem(OFFSET_STORAGE_KEY, String(offsetDeg.value))
    return
  }
  try {
    await sendPosition(currentSite.value)
  } catch {
    offsetDeg.value -= delta
  }
}

async function tare() {
  if (!hasState.value || offsetDeg.value === 0) return
  const previous = offsetDeg.value
  offsetDeg.value = 0
  if (currentSite.value === -1) {
    localStorage.setItem(OFFSET_STORAGE_KEY, '0')
    return
  }
  try {
    await sendPosition(currentSite.value)
  } catch {
    offsetDeg.value = previous
  }
}

function selectSite(index: number) {
  if (index === currentSite.value || isLoading.value || !hasState.value) return
  sendPosition(index)
}

const websocketStore = useWebsocketStore()
websocketStore.onMessage<ControllerStateMessage>('science', 'sp_controller_state', (msg) => {
  funnelState.value = msg
})
</script>

<style scoped>
.funnel-dial {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  overflow: visible;
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

.seg-target {
  font-size: 13px;
  font-weight: 500;
  fill: var(--text-muted);
  pointer-events: none;
  user-select: none;
  tab-size: 0;
  font-variant-numeric: tabular-nums;
}

.segment.active .seg-target,
.segment:hover:not(.disabled) .seg-target {
  fill: #fff;
}

.segment.disabled .seg-target {
  fill: var(--disabled-text, #9ca3af);
}

.blade {
  cursor: pointer;
}

.blade path {
  fill: color-mix(in srgb, var(--control-primary) 10%, var(--card-bg));
  stroke: var(--input-border);
  stroke-width: 1;
  transition: fill 0.15s ease;
}

.blade:hover:not(.disabled) path {
  fill: var(--control-primary);
}

.blade:hover:not(.disabled) .blade-label {
  fill: #fff;
}

.blade.disabled {
  cursor: not-allowed;
}

.blade.disabled path {
  fill: var(--disabled-bg, #f3f4f6);
  stroke: var(--disabled-border, #e5e7eb);
}

.blade.disabled .blade-label {
  fill: var(--disabled-text, #9ca3af);
}

.blade-label {
  font-size: 10px;
  font-weight: 700;
  fill: var(--control-primary);
  text-anchor: middle;
  pointer-events: none;
  user-select: none;
}

.tare-btn {
  font-size: 11px;
  font-weight: 600;
  padding: 1px 7px;
  border-radius: 4px;
  border: 1px solid var(--input-border);
  background: transparent;
  color: var(--text-primary, currentColor);
  cursor: pointer;
  transition: background 0.15s ease, color 0.15s ease;
}

.tare-btn:hover:not(:disabled) {
  background: var(--control-primary);
  color: #fff;
  border-color: var(--control-primary);
}

.tare-btn:disabled {
  opacity: 0.4;
  cursor: not-allowed;
}

.offset-readout {
  font-size: 11px;
  font-weight: 500;
  fill: var(--text-muted);
  text-anchor: middle;
  font-variant-numeric: tabular-nums;
}

.center-val {
  font-size: 20px;
  font-weight: 700;
  fill: var(--text-primary, currentColor);
  text-anchor: middle;
}
</style>
