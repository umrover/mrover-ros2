<template>
  <div class="cmd-list-item p-2 rounded border border-2" data-testid="pw-waypoint-store-item">
    <div class="d-flex justify-content-between align-items-center mb-1">
      <h5 class="cmd-list-item-title m-0" data-testid="pw-waypoint-name">{{ waypoint.name }}</h5>
      <span class="cmd-data-label">ID: {{ waypoint.id }}</span>
    </div>
    <div class="mb-2">
      <div class="input-group input-group-sm mb-1">
        <input
          class="form-control cmd-input border-2"
          v-model.number="localLat"
          :id="'lat-' + waypoint.id"
        />
        <span class="input-group-text border-2">ºN</span>
      </div>
      <div class="input-group input-group-sm">
        <input
          class="form-control cmd-input border-2"
          v-model.number="localLon"
          :id="'lon-' + waypoint.id"
        />
        <span class="input-group-text border-2">ºW</span>
      </div>
    </div>
    <div class="d-flex gap-1">
      <button class="btn btn-success btn-sm border-2 cmd-btn-text" @click="addWaypoint">Add</button>
      <button
        class="btn btn-danger btn-sm border-2 cmd-btn-text"
        :disabled="waypoint.deletable === false"
        @click="$emit('delete', index)"
      >
        Delete
      </button>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import type { PropType } from 'vue'
import type { AutonWaypoint } from '@/types/waypoints'

export default defineComponent({
  name: 'WaypointStore',
  props: {
    waypoint: {
      type: Object as PropType<AutonWaypoint>,
      required: true,
    },
    index: {
      type: Number,
      required: true,
    },
  },
  emits: ['add', 'delete', 'update'],
  data() {
    return {
      localLat: this.waypoint.lat,
      localLon: this.waypoint.lon,
    }
  },
  watch: {
    localLat(newVal) {
      this.emitUpdate(newVal, this.localLon)
    },
    localLon(newVal) {
      this.emitUpdate(this.localLat, newVal)
    },
  },
  methods: {
    emitUpdate(lat: number, lon: number) {
      const updatedWaypoint = {
        ...this.waypoint,
        lat: lat,
        lon: lon,
      }
      this.$emit('update', updatedWaypoint, this.index)
    },
    addWaypoint() {
      const updatedWaypoint = {
        ...this.waypoint,
        lat: this.localLat,
        lon: this.localLon,
      }
      this.$emit('add', updatedWaypoint)
    },
  },
})
</script>

<style scoped>
.cmd-list-item {
  background-color: var(--card-bg);
}

.cmd-list-item-title {
  font-size: 0.875rem;
  font-weight: 600;
}

.cmd-data-label {
  font-size: 0.6875rem;
  color: var(--text-muted);
}

.input-group-text {
  font-size: 0.75rem;
  min-width: 40px;
  justify-content: center;
}

.cmd-btn-text {
  font-size: 0.6875rem;
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 0.03em;
  flex: 1 1 0;
}
</style>
