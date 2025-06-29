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
					v-model.number="waypoint.lat"
					:id="'lat-' + waypoint.id"
				/>
				<span class="input-group-text">ºN</span>
			</div>
			<div class="input-group mb-1">
				<input
					class="form-control"
					v-model.number="waypoint.lon"
					:id="'lon-' + waypoint.id"
				/>
				<span class="input-group-text">ºW</span>
			</div>
      <div class="waypoint-button-row">
        <button class="btn btn-success" @click="$emit('add', waypoint)">
          Add
        </button>
				<button
					class="btn btn-danger mx-1"
					:disabled="index <= 6"
					@click="$emit('delete', index)"
				>
					Delete
				</button>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import type { PropType } from 'vue'
import type { Waypoint } from '../types/waypoint'

export default {
  name: 'WaypointStore',
  props: {
    waypoint: {
      type: Object as PropType<Waypoint>,
      required: true,
    },
    index: {
      type: Number,
      required: true,
    },
  },
  emits: ['add', 'delete'],
}
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
	align-items: center;
}

.input-group-text {
	min-width: 50px;
}
</style>
