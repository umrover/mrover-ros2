<template>
  <div
    class="rounded-circle flex-shrink-0"
    :class="isActive ? 'bg-success' : 'bg-danger'"
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
