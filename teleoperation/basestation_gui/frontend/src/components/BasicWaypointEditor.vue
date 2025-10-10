<template>
  <div class="wrapper d-flex m-0 p-2 justify-content-between gap-3 w-100 h-100">
    <div class="d-flex flex-column w-100 gap-2">
      <h3 class="m-0 p-0">Add Waypoint</h3>
      <div class="form-group d-flex gap-2 align-items-center">
        <label for="waypointname" class="form-label m-0 p-0">Name:</label>
        <input class="form-control" id="waypointname" v-model="name" />
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
      input: {
        lat: {
          d: 0,
        },
        lon: {
          d: 0,
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
        lat: { d: number}
        lon: { d: number}
      },
      isDrone: boolean,
    ) {
      this.storedWaypoints.push({
        name: this.name,
        lat: (coord.lat.d).toFixed(5),
        lon: (coord.lon.d).toFixed(5),
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

    clickPoint: function (newClickPoint) {
      this.input.lat.d = newClickPoint.lat
      this.input.lon.d = newClickPoint.lon
    },
  },

  created: function () {
    this.setHighlightedWaypoint(-1)
    this.setSearchWaypoint(-1)
    this.setWaypointList([])

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

    formatted_odom: function () {
      return {
        lat: this.odom.latitude_deg,
        lon: this.odom.longitude_deg,
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
