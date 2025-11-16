<template>
  <div class="waypoints p-2 mb-2">
    <div class="waypoint-header mb-1">
      <h5 class="mb-0">{{ waypoint.name }}</h5>
      <small class="text-muted">ID: {{ waypoint.id }}</small>
    </div>
    <div>
      <div class="input-group mb-1">
        <input
          class="form-control"
          v-model.number="localLat" :id="'lat-' + waypoint.id"
        />
        <span class="input-group-text">ºN</span>
      </div>
      <div class="input-group mb-1">
        <input
          class="form-control"
          v-model.number="localLon" :id="'lon-' + waypoint.id"
        />
        <span class="input-group-text">ºW</span>
      </div>
      <div class="waypoint-button-row">
        <button class="btn btn-success" @click="addWaypoint">Add</button>
        <button
          class="btn btn-danger"
          :disabled="index <= 7"
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
  background-color: #ffffff;
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
