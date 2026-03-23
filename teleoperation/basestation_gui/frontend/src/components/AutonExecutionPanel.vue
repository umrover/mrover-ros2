<template>
  <div class="waypoint-wrapper p-2 rounded grow overflow-auto relative">
    <div v-if="autonomyStore.execution.length === 0" class="course-empty-state">
      <i class="bi bi-cursor"></i>
      <span>No active waypoints</span>
    </div>
    <div
      v-for="(wp, index) in autonomyStore.execution"
      :key="wp.db_id ?? index"
      class="list-item"
      :class="{ 'kbd-highlighted': highlightedIndex === index }"
    >
      <div class="flex justify-between items-center mb-1">
        <h5 class="list-item-title">{{ wp.name }}</h5>
        <span v-if="wp.tag_id != null" class="data-label">#{{ wp.tag_id }}</span>
      </div>
      <div class="flex justify-between items-center">
        <small class="text-muted">{{ wp.lat.toFixed(6) }}N, {{ wp.lon.toFixed(6) }}W</small>
        <div class="flex gap-1">
          <button class="btn btn-sm btn-warning btn-icon" :disabled="autonomyStore.isNavigating" @click="autonomyStore.unstageOne(wp)"><i class="bi bi-box-arrow-left" /></button>
          <button class="btn btn-sm btn-danger btn-icon" :disabled="autonomyStore.isNavigating" @click="autonomyStore.removeFromExecution(wp)"><i class="bi bi-trash-fill" /></button>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { useAutonomyStore } from '@/stores/autonomy'

defineProps<{
  highlightedIndex?: number
}>()

const autonomyStore = useAutonomyStore()
</script>

<style scoped>
.waypoint-wrapper {
  scrollbar-gutter: stable;
  background-color: var(--view-bg);
}

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
