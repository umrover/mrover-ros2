<template>
  <div
    class="shrink-0 rounded-full"
    :class="isActive ? 'bg-cmd-success' : 'bg-cmd-danger'"
    :style="{ width: `${size}px`, height: `${size}px` }"
  ></div>
</template>

<script lang="ts" setup>
interface Props {
  isActive: boolean
  size?: number
}

withDefaults(defineProps<Props>(), {
  size: 16
})
</script>

