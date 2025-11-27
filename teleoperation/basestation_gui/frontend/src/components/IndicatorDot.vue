<template>
  <div
    class="rounded-circle"
    :class="isActive ? 'bg-success' : 'bg-danger'"
    :style="{ width: `${size}px`, height: `${size}px` }"
  ></div>
</template>

<script lang="ts" setup>
import { defineProps, withDefaults } from 'vue'

interface Props {
  isActive: boolean
  size?: number
}

withDefaults(defineProps<Props>(), {
  size: 16
})
</script>
