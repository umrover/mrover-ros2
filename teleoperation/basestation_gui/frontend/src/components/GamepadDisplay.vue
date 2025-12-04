<template>
  <div ref="container"></div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch } from 'vue'
import { GamepadDisplay } from 'gamepad-display'

const props = defineProps<{
  axes: number[]
  buttons: number[]
}>()

const container = ref<HTMLElement | null>(null)
let display: GamepadDisplay | null = null

onMounted(() => {
  if (container.value) {
    display = new GamepadDisplay(container.value)

    if (props.axes && props.buttons) {
      display.update(props.axes, props.buttons)
    }
  }
})

watch(
  () => [props.axes, props.buttons],
  () => {
    if (display && props.axes && props.buttons) {
      display.update(props.axes, props.buttons)
    }
  },
  { deep: true }
)

onUnmounted(() => {
  display?.destroy()
})
</script>
