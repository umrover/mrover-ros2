<template>
  <div
    class="shrink-0 rounded-full"
<<<<<<< HEAD
    :class="isActive ? 'bg-cmd-success' : 'bg-cmd-danger'"
=======
    :class="isActive ? 'bg-success' : 'bg-danger'"
>>>>>>> origin/main
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

