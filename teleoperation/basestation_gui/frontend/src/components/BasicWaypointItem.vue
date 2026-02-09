<template>
  <div class="cmd-list-item border border-2 rounded p-2" :class="{ 'cmd-list-item--drone': waypoint.drone }">
    <div class="d-flex justify-content-between align-items-center mb-1">
      <h5 class="m-0">{{ waypoint.name }}</h5>
      <span class="cmd-data-label">#{{ index }}</span>
    </div>

    <div class="d-flex align-items-baseline gap-1 mb-2">
      <span class="cmd-data-value waypoint-coord">{{ waypoint.lat.toFixed(5) }}</span>
      <span class="cmd-data-unit">N</span>
      <span class="text-muted mx-1">|</span>
      <span class="cmd-data-value waypoint-coord">{{ waypoint.lon.toFixed(5) }}</span>
      <span class="cmd-data-unit">W</span>
    </div>

    <div class="d-flex gap-1">
      <button class="btn btn-success btn-sm border-2 flex-fill" @click="$emit('find', { index: index })">Find</button>
      <button class="btn btn-success btn-sm border-2 flex-fill" @click="$emit('search', {index: index})">Search</button>
      <button class="btn btn-outline-danger btn-sm border-2 flex-fill" @click="$emit('delete', { index: index })">Delete</button>
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
.cmd-list-item {
  background-color: var(--card-bg);
}

.waypoint-coord {
  font-size: 0.9375rem;
}

.cmd-list-item--drone {
  border-color: var(--cmd-status-info) !important;
  background-color: rgba(30, 102, 245, 0.05);
}

[data-bs-theme='dark'] .cmd-list-item--drone {
  background-color: rgba(137, 180, 250, 0.1);
}
</style>