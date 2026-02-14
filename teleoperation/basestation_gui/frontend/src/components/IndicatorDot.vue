<template>
  <div
    class="shrink-0 rounded-full"
    :class="isActive === null ? '' : isActive ? 'bg-cmd-success' : 'bg-cmd-danger'"
    :style="{ width: `${size}px`, height: `${size}px`, backgroundColor: isActive === null ? 'var(--text-muted)' : undefined }"
  ></div>
</template>

<script lang="ts" setup>
interface Props {
  isActive: boolean | null
  size?: number
}

withDefaults(defineProps<Props>(), {
  size: 16
})
</script>

