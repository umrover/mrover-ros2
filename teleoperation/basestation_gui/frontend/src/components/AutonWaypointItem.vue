<template>
  <div class="cmd-list-item p-2 rounded border border-2">
    <div class="d-flex justify-content-between align-items-center mb-1">
      <div class="d-flex align-items-center gap-2">
        <i class="bi bi-grip-vertical drag-handle"></i>
        <h5 class="cmd-list-item-title m-0">{{ waypoint.name }}</h5>
      </div>
      <span class="cmd-data-label">#{{ waypoint.id }}</span>
    </div>

    <div class="d-flex align-items-center justify-content-center gap-1 mb-2">
      <span class="cmd-data-value">{{ waypoint.lat.toFixed(7) }}</span>
      <span class="cmd-data-unit">ºN</span>
      <span class="text-muted mx-1">|</span>
      <span class="cmd-data-value">{{ waypoint.lon.toFixed(7) }}</span>
      <span class="cmd-data-unit">ºW</span>
    </div>

    <div class="d-flex gap-1">
      <button
        class="btn btn-sm border-2 flex-fill"
        :class="enable_costmap ? 'btn-success' : 'btn-danger'"
        @click="toggleCostmap"
      >
        Costmap
      </button>
      <button class="btn btn-sm btn-danger border-2 flex-fill" @click="deleteWaypoint">Delete</button>
    </div>
  </div>
</template>

<script lang="ts">
import Vuex from 'vuex'
const { mapGetters } = Vuex
import { convertDMS } from '../utils/map'

export default {
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
    ...mapGetters('map', {
      odom_format: 'odomFormat',
    }),

    ...mapGetters('autonomy', {
      highlightedWaypoint: 'highlightedWaypoint',
    }),

    min_enabled: function () {
      return this.odom_format != 'D'
    },

    sec_enabled: function () {
      return this.odom_format == 'DMS'
    },

    output: function () {
      return {
        lat: convertDMS({ d: this.waypoint.lat, m: 0, s: 0 }, this.odom_format),
        lon: convertDMS({ d: this.waypoint.lon, m: 0, s: 0 }, this.odom_format),
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

.cmd-data-value {
  font-size: 0.8125rem;
  font-weight: 600;
}

.cmd-data-unit {
  font-size: 0.6875rem;
  color: var(--text-muted);
}

.drag-handle {
  cursor: grab;
  color: var(--text-muted);
}

.drag-handle:active {
  cursor: grabbing;
}
</style>
