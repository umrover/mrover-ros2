<template>
  <div class="list-item" :class="{ 'list-item--drone': waypoint.drone }">
    <div class="flex justify-between items-center mb-1">
      <div class="flex items-center gap-2">
        <i class="bi bi-grip-vertical drag-handle"></i>
        <h5>{{ waypoint.name }}</h5>
      </div>
      <span class="data-label">#{{ index }}</span>
    </div>

    <div class="flex items-baseline gap-1 mb-2">
      <span class="data-value waypoint-coord">{{ waypoint.lat.toFixed(5) }}</span>
      <span class="data-unit">N</span>
      <span class="text-muted mx-1">|</span>
      <span class="data-value waypoint-coord">{{ waypoint.lon.toFixed(5) }}</span>
      <span class="data-unit">W</span>
    </div>

    <div class="flex gap-1">
      <button class="btn btn-success btn-sm flex-1" @click="$emit('find', { index: index })">Find</button>
      <button class="btn btn-success btn-sm flex-1" @click="$emit('search', {index: index})">Search</button>
      <button class="btn btn-sm btn-outline-primary flex-1" @click="$emit('delete', { index: index })">Delete</button>
    </div>
  </div>
</template>

<script lang="ts" setup>
defineProps({
  waypoint: {
    type: Object,
    required: true
  },
  index: {
    type: Number,
    required: true
  }
})
</script>

<style scoped>
.waypoint-coord {
  font-size: 0.9375rem;
}

.list-item--drone {
  background-color: rgba(var(--accent-rgb), 0.05);
  border-color: var(--accent);
}
</style>
