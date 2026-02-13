<template>
  <div class="cmd-list-item" :class="{ 'cmd-list-item--drone': waypoint.drone }">
    <div class="flex justify-between items-center mb-1">
      <h5>{{ waypoint.name }}</h5>
      <span class="cmd-data-label">#{{ index }}</span>
    </div>

    <div class="flex items-baseline gap-1 mb-2">
      <span class="cmd-data-value waypoint-coord">{{ waypoint.lat.toFixed(5) }}</span>
      <span class="cmd-data-unit">N</span>
      <span class="text-muted mx-1">|</span>
      <span class="cmd-data-value waypoint-coord">{{ waypoint.lon.toFixed(5) }}</span>
      <span class="cmd-data-unit">W</span>
    </div>

    <div class="flex gap-1">
      <button class="cmd-btn cmd-btn-success cmd-btn-sm flex-1" @click="$emit('find', { index: index })">Find</button>
      <button class="cmd-btn cmd-btn-success cmd-btn-sm flex-1" @click="$emit('search', {index: index})">Search</button>
      <button class="cmd-btn cmd-btn-sm cmd-btn-outline-primary flex-1" @click="$emit('delete', { index: index })">Delete</button>
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

.cmd-list-item--drone {
  background-color: rgb(30 102 245 / 5%);
  border-color: var(--cmd-status-info) !important;
}

:is([data-theme='dark'], [data-theme='high-contrast-dark']) .cmd-list-item--drone {
  background-color: rgb(137 180 250 / 10%);
}
</style>
