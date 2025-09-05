<template>
  <div class="wrapper d-flex m-0 p-2 justify-content-between gap-3 w-100 h-100">
    <div class="d-flex flex-column w-100 gap-2">
      <h3 class="m-0 p-0">Add Waypoint</h3>
      <div class="form-group d-flex gap-2 align-items-center">
        <label for="waypointname" class="form-label m-0 p-0">Name:</label>
        <input class="form-control" id="waypointname" v-model="name" />
      </div>

      <div class="btn-group" role="group" aria-label="Coordinate Format Selection">
        <input
          type="radio"
          class="btn-check"
          v-model="odom_format_in"
          id="radioD"
          value="D"
          autocomplete="off"
        />
        <label class="btn btn-outline-primary" for="radioD">D</label>
        <input
          type="radio"
          class="btn-check"
          v-model="odom_format_in"
          id="radioDM"
          value="DM"
          autocomplete="off"
        />
        <label class="btn btn-outline-primary" for="radioDM">DM</label>
        <input
          type="radio"
          class="btn-check"
          v-model="odom_format_in"
          id="radioDMS"
          value="DMS"
          autocomplete="off"
        />
        <label class="btn btn-outline-primary" for="radioDMS">DMS</label>
      </div>

      <div class="d-flex gap-2">
        <div class="d-flex flex-column border border-2 rounded p-2">
          <div class="d-flex justify-content-between">
            <label class="form-label">Latitude:</label>
            <div class="col-auto">N</div>
          </div>
          <div class="col input-group">
            <input class="form-control" id="deg1" v-model.number="input.lat.d" />
            <span class="input-group-text font-monospace">º</span>
          </div>
          <div v-if="min_enabled" class="col input-group">
            <input class="form-control" id="min1" v-model.number="input.lat.m" />
            <span class="input-group-text font-monospace">'</span>
          </div>
          <div v-if="sec_enabled" class="col input-group">
            <input class="form-control" id="sec1" v-model.number="input.lat.s" />
            <span class="input-group-text font-monospace">"</span>
          </div>
        </div>
        <div class="d-flex flex-column border border-2 rounded p-2">
          <div class="d-flex justify-content-between">
            <label class="form-label">Longitude:</label>
            <div class="col-auto">W</div>
          </div>
          <div class="col input-group">
            <input class="form-control" id="deg2" v-model.number="input.lon.d" />
            <span class="input-group-text font-monospace">º</span>
          </div>
          <div v-if="min_enabled" class="col input-group">
            <input class="form-control" id="min2" v-model.number="input.lon.m" />
            <span class="input-group-text font-monospace">'</span>
          </div>
          <div v-if="sec_enabled" class="col input-group">
            <input class="form-control" id="sec2" v-model.number="input.lon.s" />
            <span class="input-group-text font-monospace">"</span>
          </div>
        </div>
      </div>

      <div class="d-flex flex-column gap-2">
        <button class="btn btn-success" @click="addWaypoint(input, false)">
          Add Waypoint
        </button>
        <button
          class="btn btn-success"
          @click="addWaypoint(formatted_odom, false)"
        >
          Drop Waypoint at Rover
        </button>
        <button
          v-if="droneWaypointButton"
          class="btn btn-info"
          @click="addWaypoint(input, true)"
        >
          Add Drone Position
        </button>
      </div>
    </div>

    <div class="d-flex flex-column w-100">
      <div class="d-flex mb-2 align-items-center justify-content-between">
        <h3 class="m-0 p-0">Current Course</h3>
        <button class="btn btn-danger" @click="clearWaypoint">Clear</button>
      </div>
      <div class="waypoint-wrapper overflow-y-scroll d-flex flex-column gap-2">
        <WaypointItem
          v-for="(waypoint, i) in storedWaypoints"
          :key="i"
          :waypoint="waypoint"
          :index="i"
          @delete="deleteItem($event)"
          @find="findWaypoint($event)"
          @search="searchForWaypoint($event)"
        />
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { convertDMS } from '../utils/map.js'
import WaypointItem from './BasicWaypointItem.vue'
import Vuex from 'vuex'
const { mapMutations, mapGetters, mapActions, mapState } = Vuex
import L from 'leaflet'
import type { WebSocketState } from '../types/websocket.js'

export default {
  props: {
    odom: {
      type: Object,
      default: () => ({ latitude_deg: 0, longitude_deg: 0, bearing_deg: 0 }),
    },
    droneWaypointButton: {
      type: Boolean,
      required: false,
    },
  },

  data() {
    return {
      name: 'Waypoint',
      odom_format_in: 'DM',
      input: {
        lat: {
          d: 0,
          m: 0,
          s: 0,
        },
        lon: {
          d: 0,
          m: 0,
          s: 0,
        },
      },

      storedWaypoints: [],
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    ...mapMutations('erd', {
      setWaypointList: 'setWaypointList',
      setHighlightedWaypoint: 'setHighlightedWaypoint',
      setSearchWaypoint: 'setSearchWaypoint',
    }),

    ...mapMutations('map', {
      setOdomFormat: 'setOdomFormat',
    }),

    deleteItem: function (payload: { index: number }) {
      if (this.highlightedWaypoint == payload.index) {
        this.setHighlightedWaypoint(-1)
      }
      if (this.searchWaypoint == payload.index) {
        this.setSearchWaypoint(-1)
      }
      this.storedWaypoints.splice(payload.index, 1)
    },

    addWaypoint: function (
      coord: {
        lat: { d: number; m: number; s: number }
        lon: { d: number; m: number; s: number }
      },
      isDrone: boolean,
    ) {
      this.storedWaypoints.push({
        name: this.name,
        lat: (coord.lat.d + coord.lat.m / 60 + coord.lat.s / 3600).toFixed(5),
        lon: (coord.lon.d + coord.lon.m / 60 + coord.lon.s / 3600).toFixed(5),
        drone: isDrone,
      })
    },

    findWaypoint: function (payload: { index: number }) {
      if (payload.index === this.highlightedWaypoint) {
        this.setHighlightedWaypoint(-1)
      } else {
        this.setHighlightedWaypoint(payload.index)
      }
    },

    searchForWaypoint: function (payload: { index: number }) {
      if (payload.index === this.searchWaypoint) {
        this.setSearchWaypoint(-1)
      } else {
        this.setSearchWaypoint(payload.index)
      }
    },

    clearWaypoint: function () {
      this.storedWaypoints = []
    },
  },

  watch: {
    storedWaypoints: {
      handler: function (newList) {
        const waypoints = newList.map(
          (waypoint: {
            lat: number
            lon: number
            name: string
            drone: boolean
          }) => {
            return {
              latLng: L.latLng(waypoint.lat, waypoint.lon),
              name: waypoint.name,
              drone: waypoint.drone,
            }
          },
        )
        this.setWaypointList(waypoints)
        this.$store.dispatch('websocket/sendMessage', {
          id: 'waypoints',
          message: {
            type: 'save_basic_waypoint_list',
            data: newList,
          },
        })
      },
      deep: true,
    },

    navMessage: {
      handler: function (msg) {
        if (msg.type == 'get_basic_waypoint_list') {
          this.storedWaypoints = msg.data
          const waypoints = msg.data.map(
            (waypoint: { lat: number; lon: number; name: string }) => {
              const lat = waypoint.lat
              const lon = waypoint.lon
              return { latLng: L.latLng(lat, lon), name: waypoint.name }
            },
          )
          this.setWaypointList(waypoints)
        }
      },
      deep: true,
    },

    odom_format_in: function (newOdomFormat) {
      this.setOdomFormat(newOdomFormat)
      this.input.lat = convertDMS(this.input.lat, newOdomFormat)
      this.input.lon = convertDMS(this.input.lon, newOdomFormat)
    },

    clickPoint: function (newClickPoint) {
      this.input.lat.d = newClickPoint.lat
      this.input.lon.d = newClickPoint.lon
      this.input.lat.m = 0
      this.input.lon.m = 0
      this.input.lat.s = 0
      this.input.lon.s = 0
      this.input.lat = convertDMS(this.input.lat, this.odom_format_in)
      this.input.lon = convertDMS(this.input.lon, this.odom_format_in)
    },
  },

  created: function () {
    this.setHighlightedWaypoint(-1)
    this.setSearchWaypoint(-1)
    this.setWaypointList([])

    this.odom_format_in = this.odom_format

    window.setTimeout(() => {
      this.$store.dispatch('websocket/sendMessage', {
        id: 'waypoints',
        message: {
          type: 'get_basic_waypoint_list',
        },
      })
    }, 250)
  },

  computed: {
    ...mapState('websocket', {
      waypointsMessage: (state: WebSocketState) => state.messages['waypoints'],
    }),
    ...mapGetters('erd', {
      highlightedWaypoint: 'highlightedWaypoint',
      searchWaypoint: 'searchWaypoint',
      clickPoint: 'clickPoint',
    }),

    ...mapGetters('map', {
      odom_format: 'odomFormat',
    }),

    min_enabled: function () {
      return this.odom_format != 'D'
    },

    sec_enabled: function () {
      return this.odom_format == 'DMS'
    },

    formatted_odom: function () {
      return {
        lat: convertDMS(
          { d: this.odom.latitude_deg, m: 0, s: 0 },
          this.odom_format,
        ),
        lon: convertDMS(
          { d: this.odom.longitude_deg, m: 0, s: 0 },
          this.odom_format,
        ),
      }
    },
  },

  components: {
    WaypointItem,
  },
}
</script>

<style scoped>
.waypoint-wrapper {
  flex: 1;
  overflow-y: auto;
  background-color: #dddddd;
  padding: 8px;
  border-radius: 8px;
}

.waypoint-col {
  min-width: 300px;
}
</style>
