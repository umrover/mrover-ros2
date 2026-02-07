<template>
  <div class="cmd-list-item p-2 rounded border border-2 mb-1" data-testid="pw-waypoint-store-item">
    <div class="d-flex justify-content-between align-items-center mb-1">
      <h5 class="cmd-list-item-title m-0" data-testid="pw-waypoint-name">{{ waypoint.name }}</h5>
      <div class="d-flex align-items-center gap-2">
        <span class="cmd-data-label">Tag ID: {{ waypoint.tag_id }}</span>
        <span class="cmd-data-label">R: {{ waypoint.coverage_radius }}</span>
      </div>
    </div>
    <div>
      <div class="input-group mb-1">
        <input
          class="form-control"
          v-model.number="localLat" :id="'lat-' + waypoint.tag_id"
        />
        <span class="input-group-text">ºN</span>
      </div>
      <div class="input-group mb-1">
        <input
          class="form-control"
          v-model.number="localLon" :id="'lon-' + waypoint.tag_id"
        />
        <span class="input-group-text">ºW</span>
      </div>
      <div class="waypoint-button-row">
        <button class="btn btn-success" @click="addWaypoint">Add</button>
        <button
          class="btn btn-danger"
          :disabled="waypoint.deletable === false"
          @click="$emit('delete', index)"
        >
          Delete
        </button>
      </div>
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
.waypoints {
  background-color: var(--card-bg);
  border-radius: 6px;
  box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
}

.waypoint-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.waypoint-button-row {
  display: flex;
  justify-content: space-between;
  gap: 0.5rem;
}

.waypoint-button-row .btn {
  flex: 1;
}

.input-group-text {
  min-width: 50px;
}
</style>
