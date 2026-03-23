<template>
  <div class="list-item" :class="{ 'kbd-highlighted': highlighted }" data-testid="pw-route-item">
    <div class="flex justify-between items-center mb-1">
      <div class="flex items-center gap-2">
        <i class="bi bi-grip-vertical drag-handle"></i>
        <h5 class="list-item-title">{{ waypoint.name }}</h5>
      </div>
      <span v-if="waypoint.tag_id != null" class="data-label">#{{ waypoint.tag_id }}</span>
    </div>

    <div class="flex justify-between items-center">
      <small class="text-muted">{{ waypoint.lat.toFixed(6) }}N, {{ waypoint.lon.toFixed(6) }}W</small>
      <div class="flex gap-1">
        <button class="btn btn-success btn-sm btn-icon" @click="$emit('stage', waypoint)"><i class="bi bi-plus-lg"></i></button>
        <button class="btn btn-sm btn-danger btn-icon" @click="$emit('delete', { waypoint })"><i class="bi bi-trash-fill"></i></button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import type { AutonWaypoint } from '@/types/waypoints'

defineProps<{
  waypoint: AutonWaypoint
  highlighted?: boolean
}>()

defineEmits<{
  stage: [waypoint: AutonWaypoint]
  delete: [payload: { waypoint: AutonWaypoint }]
}>()
</script>

<style scoped>
.list-item-title {
  font-size: 0.875rem;
  font-weight: 600;
}

.data-label {
  font-size: 0.6875rem;
  color: var(--text-muted);
}

.btn-icon {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 1.75rem;
  height: 1.75rem;
  padding: 0;
  font-size: 0.75rem;
}
</style>
