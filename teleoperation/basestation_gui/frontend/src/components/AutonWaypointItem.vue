<template>
  <div class="waypoints p-2">
    <div class="waypoint-header mb-1">
      <h5 class="mb-0">{{ waypoint.name }}</h5>
      <small class="text-muted">ID: {{ waypoint.id }}</small>
    </div>

    <div class="coordinates mb-1">
      <p class="coordinate m-0">{{ waypoint.lat.toFixed(7) }}ºN</p>
      <span class="mx-2">|</span>
      <p class="coordinate m-0">{{ waypoint.lon.toFixed(7) }}ºW</p>
    </div>

    <div class="d-flex w-100 gap-2">
      <button
        v-if="!enable_costmap"
        class="btn btn-danger w-100"
        @click="toggleCostmap"
      >
        Costmap
      </button>
      <button
        v-if="enable_costmap"
        class="btn btn-success w-100"
        @click="toggleCostmap"
      >
        Costmap
      </button>
      <button class="btn btn-danger w-100" @click="deleteWaypoint">Delete</button>
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
</style>
