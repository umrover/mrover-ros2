<template>
  <div class="cmd-list-item p-2 rounded border border-2">
    <div class="d-flex justify-content-between align-items-center mb-1">
      <div class="d-flex align-items-center gap-2">
        <i class="bi bi-grip-vertical drag-handle"></i>
        <h5 class="cmd-list-item-title m-0">{{ waypoint.name }}</h5>
      </div>
      <span class="cmd-data-label">#{{ waypoint.id }}</span>
    </div>

    <div class="d-flex justify-content-between align-items-center">
      <small class="text-muted">{{ waypoint.lat.toFixed(6) }}N, {{ waypoint.lon.toFixed(6) }}W</small>
      <div class="d-flex gap-1">
        <button
          class="btn btn-sm border-2 cmd-btn-text"
          :class="enable_costmap ? 'btn-success' : 'btn-danger'"
          @click="toggleCostmap"
        >Costmap</button>
        <button class="btn btn-sm btn-danger border-2 cmd-btn-icon" @click="deleteWaypoint"><i class="bi bi-trash-fill"></i></button>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { useAutonomyStore } from '@/stores/autonomy'

export default {
  setup() {
    const autonomyStore = useAutonomyStore()
    return { autonomyStore }
  },

  props: {
    waypoint: {
      type: Object,
      required: true,
    },
  },

  data() {
    return {
      enable_costmap: true,
    }
  },

  methods: {
    toggleCostmap() {
      this.enable_costmap = !this.enable_costmap
      this.$emit('toggleCostmap', {
        waypoint: this.waypoint,
        enable_costmap: this.enable_costmap,
      })
    },

    deleteWaypoint() {
      this.$emit('delete', { waypoint: this.waypoint })
    },
  },

  watch: {
    'waypoint.enable_costmap': {
      immediate: true,
      handler(newVal) {
        this.enable_costmap = newVal
      },
    },
  },

  computed: {
    highlightedWaypoint() {
      return this.autonomyStore.highlightedWaypoint
    },

    output: function () {
      return {
        lat: this.waypoint.lat,
        lon: this.waypoint.lon,
      }
    },
  },
}
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

.cmd-btn-icon {
  width: 1.75rem;
  height: 1.75rem;
  padding: 0;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  font-size: 0.75rem;
}

.cmd-btn-text {
  font-size: 0.6875rem;
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 0.03em;
  padding: 0.15rem 0.4rem;
}

.drag-handle {
  cursor: grab;
  color: var(--text-muted);
}

.drag-handle:active {
  cursor: grabbing;
}
</style>
