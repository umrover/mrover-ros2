<template>
  <div
    class="waypoints p-2"
    v-bind:class="{ 'drone-waypoint': waypoint.drone }"
  >
    <div class="waypoint-header mb-1">
      <h5 class="mb-0 w-100">{{ waypoint.name }}</h5>
    </div>

    <div class="coordinates mb-2 w-100">
      <p class="coordinate m-0">{{ waypoint.lat }}ºN</p>
      <span class="mx-2">|</span>
      <p class="coordinate m-0">{{ waypoint.lon }}ºW</p>
    </div>

    <div class="d-flex w-100 gap-2">
      <button class="btn btn-success btn-sm w-100" @click="$emit('find', { index: index })">Find</button>
      <button class="btn btn-success btn-sm w-100" @click="$emit('search', {index: index})">Search</button>
      <button class="btn btn-danger btn-sm w-100" @click="$emit('delete', { index: index })">Delete</button>
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
.waypoints {
  background-color: #ffffff;
  border-radius: 6px;
  box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
}

.waypoint-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.coordinates {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}

.coordinate {
  text-align: center;
  flex: 1;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.drone-waypoint {
  background-color: #bbeeff;
  border: 1px solid cyan;
}
</style>