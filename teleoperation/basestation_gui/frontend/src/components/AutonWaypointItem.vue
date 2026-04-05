<template>
  <div class="list-item" data-testid="pw-route-item">
    <div class="flex justify-between items-center mb-1">
      <div class="flex items-center gap-2">
        <i class="bi bi-grip-vertical drag-handle"></i>
        <h5 class="list-item-title">{{ waypoint.name }}</h5>
      </div>
      <span class="data-label">#{{ waypoint.tag_id }}</span>
    </div>

    <div class="flex justify-between items-center">
      <small class="text-muted">{{ waypoint.lat.toFixed(6) }}N, {{ waypoint.lon.toFixed(6) }}W</small>
      <div class="flex gap-1">
        <button
          class="btn btn-sm btn-text"
          :class="waypoint.enable_costmap ? 'btn-success' : 'btn-danger'"
          data-testid="pw-route-costmap-toggle"
          @click="toggleCostmap"
        >Costmap</button>
        <button class="btn btn-sm btn-danger btn-icon" @click="$emit('delete', { waypoint })"><i class="bi bi-trash-fill"></i></button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import type { AutonWaypoint } from '@/types/waypoints'

const props = defineProps<{
  waypoint: AutonWaypoint
}>()

const emit = defineEmits<{
  delete: [payload: { waypoint: AutonWaypoint }]
  toggleCostmap: [payload: { waypoint: AutonWaypoint; enable_costmap: boolean }]
}>()

function toggleCostmap() {
  emit('toggleCostmap', {
    waypoint: props.waypoint,
    enable_costmap: !props.waypoint.enable_costmap,
  })
}
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

.btn-text {
  padding: 0.15rem 0.4rem;
  font-size: 0.6875rem;
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 0.03em;
}

</style>
