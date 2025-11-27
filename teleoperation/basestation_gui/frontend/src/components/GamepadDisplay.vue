<template>
  <svg viewBox="0 0 225 120" class="gamepad-svg">
    <g class="bumpers-triggers">
      <rect :x="bumpersTriggersOrigin.x + 0" :y="bumpersTriggersOrigin.y + 0" width="40" height="10" rx="2"
        :class="{ pressed: buttonPressed(4) }" class="button-outline" />
      <rect :x="bumpersTriggersOrigin.x + 50" :y="bumpersTriggersOrigin.y + 0" width="40" height="10" rx="2"
        :class="{ pressed: buttonPressed(5) }" class="button-outline" />

      <rect :x="bumpersTriggersOrigin.x + 0" :y="bumpersTriggersOrigin.y + 15" width="40" height="15" rx="2"
        class="button-outline" />
      <rect :x="bumpersTriggersOrigin.x + 0" :y="bumpersTriggersOrigin.y + 15" width="40" height="15" rx="2"
        class="button-filled"
        :style="{ clipPath: `inset(0 0 0 ${(1 - buttons[6]) * 100}%)` }" />

      <rect :x="bumpersTriggersOrigin.x + 50" :y="bumpersTriggersOrigin.y + 15" width="40" height="15" rx="2"
        class="button-outline" />
      <rect :x="bumpersTriggersOrigin.x + 50" :y="bumpersTriggersOrigin.y + 15" width="40" height="15" rx="2"
        class="button-filled"
        :style="{ clipPath: `inset(0 ${(1 - buttons[7]) * 100}% 0 0)` }" />
    </g>

    <circle :cx="leftStickOrigin.x + 0" :cy="leftStickOrigin.y + 0" r="22" class="stick-base"/>
    <g class="left-stick" :transform="`translate(${axes[0] * 10}, ${axes[1] * 10})`">
      <circle :cx="leftStickOrigin.x + 0" :cy="leftStickOrigin.y + 0" r="12"
        :class="{ pressed: buttonPressed(10) }" class="button-outline" />
    </g>

    <g class="dpad">
      <rect :x="dpadOrigin.x + 15" :y="dpadOrigin.y + 0" width="12" height="15" rx="2"
        :class="{ pressed: buttonPressed(12) }" class="button-outline" />
      <rect :x="dpadOrigin.x + 0" :y="dpadOrigin.y + 15" width="15" height="12" rx="2"
        :class="{ pressed: buttonPressed(14) }" class="button-outline" />
      <rect :x="dpadOrigin.x + 15" :y="dpadOrigin.y + 27" width="12" height="15" rx="2"
        :class="{ pressed: buttonPressed(13) }" class="button-outline" />
      <rect :x="dpadOrigin.x + 27" :y="dpadOrigin.y + 15" width="15" height="12" rx="2"
        :class="{ pressed: buttonPressed(15) }" class="button-outline" />
    </g>

    <g class="center-buttons">
      <circle :cx="centerButtonsOrigin.x + 0" :cy="centerButtonsOrigin.y + 0" r="5"
        :class="{ pressed: buttonPressed(8) }" class="button-outline" />
      <circle :cx="centerButtonsOrigin.x + 30" :cy="centerButtonsOrigin.y + 0" r="5"
        :class="{ pressed: buttonPressed(9) }" class="button-outline" />
    </g>

    <g class="face-buttons">
      <circle :cx="faceButtonsOrigin.x + 18" :cy="faceButtonsOrigin.y + 36" r="10"
        :class="{ pressed: buttonPressed(0) }" class="button-outline" />
      <text :x="faceButtonsOrigin.x + 18" :y="faceButtonsOrigin.y + 36" class="button-label">A</text>

      <circle :cx="faceButtonsOrigin.x + 36" :cy="faceButtonsOrigin.y + 18" r="10"
        :class="{ pressed: buttonPressed(1) }" class="button-outline" />
      <text :x="faceButtonsOrigin.x + 36" :y="faceButtonsOrigin.y + 18" class="button-label">B</text>

      <circle :cx="faceButtonsOrigin.x + 0" :cy="faceButtonsOrigin.y + 18" r="10"
        :class="{ pressed: buttonPressed(2) }" class="button-outline" />
      <text :x="faceButtonsOrigin.x + 0" :y="faceButtonsOrigin.y + 18" class="button-label">X</text>

      <circle :cx="faceButtonsOrigin.x + 18" :cy="faceButtonsOrigin.y + 0" r="10"
        :class="{ pressed: buttonPressed(3) }" class="button-outline" />
      <text :x="faceButtonsOrigin.x + 18" :y="faceButtonsOrigin.y + 0" class="button-label">Y</text>
    </g>

    <circle :cx="rightStickOrigin.x + 0" :cy="rightStickOrigin.y + 0" r="22" class="stick-base" />
    <g class="right-stick" :transform="`translate(${axes[2] * 10}, ${axes[3] * 10})`">
      <circle :cx="rightStickOrigin.x + 0" :cy="rightStickOrigin.y + 0" r="12"
        :class="{ pressed: buttonPressed(11) }" class="button-outline" />
    </g>
  </svg>
</template>

<script lang="ts" setup>
import { defineProps } from 'vue'

const props = defineProps<{
  axes: number[]
  buttons: number[]
}>()

const bumpersTriggersOrigin = { x: 65, y: 15 }
const centerButtonsOrigin = { x: 95, y: 60 }
const leftStickOrigin = { x: 30, y: 30 }
const dpadOrigin = { x: 8, y: 60 }
const faceButtonsOrigin = { x: 175, y: 15 }
const rightStickOrigin = { x: 194, y: 88 }

function buttonPressed(index: number): boolean {
  return props.buttons[index] > 0.5
}
</script>

<style scoped>
.gamepad-svg {
  width: 100%;
  max-width: 235px;
  height: auto;
}

.button-outline {
  fill: none;
  stroke: #6c757d;
  stroke-width: 2;
  transition: all 0.1s;
}

.button-outline.pressed {
  fill: #0d6efd;
  stroke: #0d6efd;
}

.button-filled {
  fill: #0d6efd;
  stroke: none;
  transition: clip-path 0.1s;
}

.stick-base {
  fill: none;
  stroke: #dee2e6;
  stroke-width: 2;
}

.button-label {
  fill: #6c757d;
  font-size: 10px;
  font-weight: 600;
  text-anchor: middle;
  dominant-baseline: middle;
  pointer-events: none;
}
</style>
