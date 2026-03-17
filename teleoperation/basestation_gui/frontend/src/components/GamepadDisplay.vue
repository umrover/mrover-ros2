<template>
  <div ref="container" class="w-full h-full"></div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch } from 'vue'
import { GamepadDisplay } from 'gamepad-display'

const props = defineProps<{
  axes: number[]
  buttons: number[]
  layout?: 'default' | 'horizontal'
}>()

const container = ref<HTMLElement | null>(null)
let display: GamepadDisplay | null = null

onMounted(() => {
  if (container.value) {
    display = new GamepadDisplay(container.value, {
      layout: props.layout ?? 'default'
    })

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
  }
)

onUnmounted(() => {
  display?.destroy()
})
</script>
