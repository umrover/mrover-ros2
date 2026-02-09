<template>
  <div
    class="indicator-dot"
    :class="isActive ? 'indicator-dot--active' : 'bg-danger'"
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

<style scoped>
.indicator-dot {
  border-radius: 50%;
  flex-shrink: 0;
}

.indicator-dot--active {
  background-color: var(--cmd-status-ok);
}
</style>
