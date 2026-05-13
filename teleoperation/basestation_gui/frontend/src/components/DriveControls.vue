<template>
  <div class="flex flex-col w-full h-full gap-2">
    <div class="flex justify-between items-center w-full">
      <h4 class="component-header">Drive Controls</h4>
      <IndicatorDot :is-active="connected" />
    </div>
    <svg viewBox="-42 -42 84 102" class="joystick-svg grow min-h-0" preserveAspectRatio="xMidYMid meet">
        <circle class="ring" r="40" />
      <line
        class="twist-path"
        :x1="-40 * Math.sin(twistAngle)" :y1="40 * Math.cos(twistAngle)"
        :x2="40 * Math.sin(twistAngle)" :y2="-40 * Math.cos(twistAngle)"
      />
      <circle class="stick-dot" :cx="dotX" :cy="dotY" r="8" />
      <line
        v-if="hasMicro"
        class="micro-line"
        :x1="baseDotX" :y1="baseDotY"
        :x2="dotX" :y2="dotY"
      />
      <g transform="translate(0, 52)">
        <rect class="throttle-track" x="-40" y="0" width="80" height="6" rx="3" />
        <rect class="throttle-fill" x="-40" y="0" :width="throttleWidth" height="6" rx="3" />
      </g>

    </svg>
  </div>
</template>

<script lang="ts" setup>
import { computed } from 'vue'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import IndicatorDot from './IndicatorDot.vue'

const { connected, axes } = useGamepadPolling({
  controllerIdFilter: 'Thrustmaster',
  topic: 'drive',
  messageType: 'joystick',
  transformAxes: (axes) => axes.map((v, i) => i === 1 ? -v : v),
})

function axis(index: number): number {
  return axes.value[index] ?? 0
}

const twistAngle = computed(() => axis(2) * Math.PI * 0.2)

const forwardBack = computed(() => axis(1))
const baseDotX = computed(() => forwardBack.value * 38 * Math.sin(twistAngle.value))
const baseDotY = computed(() => forwardBack.value * -38 * Math.cos(twistAngle.value))

const microOffsetX = computed(() => axis(4) * 6)
const microOffsetY = computed(() => axis(5) * 6)
const hasMicro = computed(() => Math.abs(axis(4)) > 0.05 || Math.abs(axis(5)) > 0.05)

const dotX = computed(() => baseDotX.value + microOffsetX.value)
const dotY = computed(() => baseDotY.value + microOffsetY.value)

const throttleNorm = computed(() => ((-axis(3)) + 1) / 2)
const throttleWidth = computed(() => Math.max(0, throttleNorm.value * 80))
</script>

<style scoped>
.joystick-svg {
  --outline: var(--text-muted);
  --base: var(--input-border);
  width: 100%;
  height: auto;
}

.ring {
  fill: none;
  stroke: var(--base);
  stroke-width: 2;
}

.twist-path {
  stroke: var(--outline);
  stroke-width: 1.5;
  stroke-dasharray: 3 3;
  transition: x1 0.1s, y1 0.1s, x2 0.1s, y2 0.1s;
}

.stick-dot {
  fill: var(--outline);
  stroke: none;
  transition: cx 0.1s, cy 0.1s;
}

.micro-line {
  stroke: var(--outline);
  stroke-width: 1;
  opacity: 0.5;
  stroke-dasharray: 2 2;
}

.throttle-track {
  fill: none;
  stroke: var(--base);
  stroke-width: 2;
}

.throttle-fill {
  fill: var(--outline);
  transition: width 0.1s;
}


</style>
