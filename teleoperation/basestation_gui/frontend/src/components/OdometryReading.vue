<template>
  <div class="d-flex flex-column w-100 wrap">
    <div class="d-flex justify-content-between w-100 flex-wrap gap-1">
      <div class="d-flex flex-column gap-2">
        <div class="border border-2 border-secondary rounded p-2">
          <h5 class="m-0 p-0 text-center">Rover Odom</h5>
          <div class="d-flex gap-3 justify-content-center">
            <div class="odomModule">
              <small class="text-muted">Latitude</small>
              <p class="mb-0">{{ formatted_odom.lat }}º</p>
            </div>
            <div class="odomModule">
              <small class="text-muted">Longitude</small>
              <p class="mb-0">{{ formatted_odom.lon }}º</p>
            </div>
          </div>
        </div>
        <div class="d-flex justify-content-center gap-3">
          <div class="odomModule">
            <small class="text-muted">Bearing</small>
            <p class="mb-0">{{ rover_bearing_deg.toFixed(2) }}º</p>
          </div>
          <div class="odomModule">
            <small class="text-muted">Altitude</small>
            <p class="mb-0">{{ rover_altitude.toFixed(2) }} m</p>
          </div>
        </div>
      </div>
      <div class="d-flex justify-content-center align-items-center p-0 m-0">
        <FlightAttitudeIndicator />
      </div>
      <div class="d-flex flex-column gap-2">
        <div class="border border-2 border-secondary rounded p-2">
          <h5 class="m-0 p-0 font-monospace text-center">Basestation Odom</h5>
          <div class="d-flex gap-3 justify-content-center">
            <div class="odomModule">
              <small class="text-muted">Latitude</small>
              <p class="mb-0">{{ formatted_basestation_odom.lat }}º</p>
            </div>
            <div class="odomModule">
              <small class="text-muted">Longitude</small>
              <p class="mb-0">{{ formatted_basestation_odom.lon }}º</p>
            </div>
          </div>
        </div>
        <div class="d-flex justify-content-center gap-3">
          <div class="odomModule">
            <small class="text-muted">Odom Status</small>
            <p class="mb-0">{{ get_odom_status }}</p>
          </div>
          <div class="odomModule">
            <small class="text-muted">Drone Status</small>
            <p class="mb-0">{{ get_drone_status }}</p>
          </div>
        </div>
      </div>
    </div>
    <div class="d-flex justify-content-center">
      <IMUCalibration />
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex
import {quaternionToMapAngle } from '../utils/map.ts'
import IMUCalibration from './IMUCalibration.vue'
import FlightAttitudeIndicator from './FlightAttitudeIndicator.vue'
import type { WebSocketState } from '../types/websocket.js'
import type {
  Odom,
  FormattedOdom,
  OdomData,
  NavMessage,
} from '../types/coordinates.js'

export default defineComponent({
  components: {
    FlightAttitudeIndicator,
    IMUCalibration,
  },

  emits: ['odom', 'drone_odom', 'basestation_odom'],

  data(): OdomData {
    return {
      rover_latitude_deg: 38.4071654,
      rover_longitude_deg: -110.7923927,
      rover_bearing_deg: 0,
      rover_altitude: 0,
      rover_status: false,
      drone_latitude_deg: 38.4071654,
      drone_longitude_deg: -110.7923927,
      drone_status: false,
      basestation_latitude_deg: 38.4071654,
      basestation_longitude_deg: -110.7923927,
      basestation_status: false,
    }
  },
  computed: {
    ...mapState('websocket', {
      navMessage: (state: WebSocketState) => state.messages['nav'],
    }),

    formatted_odom(): FormattedOdom {
      return {
        lat: this.rover_latitude_deg,
        lon: this.rover_longitude_deg,
      }
    },
    formatted_basestation_odom(): FormattedOdom {
      return {
        lat: this.basestation_latitude_deg,
        lon: this.basestation_longitude_deg,
      }
    },

    alt_available(): boolean {
      return !isNaN(this.rover_altitude)
    },
    get_odom_status(): string {
      return this.rover_status ? 'Fixed' : 'Not Fixed'
    },
    get_drone_status(): string {
      return this.drone_status ? 'Fixed' : 'Not Fixed'
    },
  },

  watch: {
    // The watcher now correctly gets the type from the manual computed property.
    navMessage(msg: NavMessage | undefined) {
      if (!msg) return

      if (msg.type === 'gps_fix') {
        this.rover_latitude_deg = msg.latitude
        this.rover_longitude_deg = msg.longitude
        this.rover_altitude = msg.altitude
        this.rover_status = msg.status
        this.$emit('odom', {
          latitude_deg: this.rover_latitude_deg,
          longitude_deg: this.rover_longitude_deg,
          bearing_deg: this.rover_bearing_deg,
        } as Odom)
      } else if (msg.type === 'basestation_position') {
        this.basestation_latitude_deg = msg.latitude
        this.basestation_longitude_deg = msg.longitude
        this.basestation_status = msg.status
        this.$emit('basestation_odom', {
          latitude_deg: this.basestation_latitude_deg,
          longitude_deg: this.basestation_longitude_deg,
        } as Odom)
      } else if (msg.type === 'drone_waypoint') {
        this.drone_latitude_deg = msg.latitude
        this.drone_longitude_deg = msg.longitude
        this.drone_status = msg.status
        this.$emit('drone_odom', {
          latitude_deg: this.drone_latitude_deg,
          longitude_deg: this.drone_longitude_deg,
        })
      } else if (msg.type === 'orientation') {
        this.rover_bearing_deg = quaternionToMapAngle(msg.orientation)
        this.$emit('odom', {
          latitude_deg: this.rover_latitude_deg,
          longitude_deg: this.rover_longitude_deg,
          bearing_deg: this.rover_bearing_deg,
        } as Odom)
      }
    },
  },
})
</script>

<style scoped>
.odomModule {
  width: 100px;
}

.wrap {
  min-width: 700px;
}
</style>
