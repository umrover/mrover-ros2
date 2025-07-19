<template>
  <div class="d-flex flex-column w-100">
    <div class="d-flex justify-content-center w-100 flex-wrap gap-4">
      <div class="d-flex flex-column gap-2">
        <div class="border border-2 border-secondary rounded p-2">
          <h5 class="m-0 p-0 text-center">Rover Odom</h5>
          <div class="d-flex gap-3 justify-content-center">
            <div class="odomModule">
              <small class="text-muted">Latitude</small>
              <p class="mb-0">{{ formatted_odom.lat.d }}º</p>
              <p v-if="min_enabled" class="mb-0">
                {{ formatted_odom.lat.m }}' N
              </p>
              <p v-if="sec_enabled" class="mb-0">
                {{ formatted_odom.lat.s }}" N
              </p>
            </div>
            <div class="odomModule">
              <small class="text-muted">Longitude</small>
              <p class="mb-0">{{ formatted_odom.lon.d }}º</p>
              <p v-if="min_enabled" class="mb-0">
                {{ formatted_odom.lon.m }}' E
              </p>
              <p v-if="sec_enabled" class="mb-0">
                {{ formatted_odom.lon.s }}" E
              </p>
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
              <p class="mb-0">{{ formatted_basestation_odom.lat.d }}º</p>
              <p v-if="min_enabled" class="mb-0">
                {{ formatted_basestation_odom.lat.m }}' N
              </p>
              <p v-if="sec_enabled" class="mb-0">
                {{ formatted_basestation_odom.lat.s }}" N
              </p>
            </div>
            <div class="odomModule">
              <small class="text-muted">Longitude</small>
              <p class="mb-0">{{ formatted_basestation_odom.lon.d }}º</p>
              <p v-if="min_enabled" class="mb-0">
                {{ formatted_basestation_odom.lon.m }}' E
              </p>
              <p v-if="sec_enabled" class="mb-0">
                {{ formatted_basestation_odom.lon.s }}" E
              </p>
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
import { convertDMS, quaternionToMapAngle } from '../utils/map.js'
import IMUCalibration from './IMUCalibration.vue'
import FlightAttitudeIndicator from './FlightAttitudeIndicator.vue'
import type { WebSocketState } from '../types/websocket.js'
import type {
  Odom,
  FormattedOdom,
  OdomData,
  NavMessage,
} from '../types/coordinates'

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

    odom_format(): string {
      return this.$store.getters['map/odomFormat']
    },

    formatted_odom(): FormattedOdom {
      return {
        lat: convertDMS(
          { d: this.rover_latitude_deg, m: 0, s: 0 },
          this.odom_format,
        ),
        lon: convertDMS(
          { d: this.rover_longitude_deg, m: 0, s: 0 },
          this.odom_format,
        ),
      }
    },
    formatted_basestation_odom(): FormattedOdom {
      return {
        lat: convertDMS(
          { d: this.basestation_latitude_deg, m: 0, s: 0 },
          this.odom_format,
        ),
        lon: convertDMS(
          { d: this.basestation_longitude_deg, m: 0, s: 0 },
          this.odom_format,
        ),
      }
    },
    min_enabled(): boolean {
      return this.odom_format !== 'D'
    },
    sec_enabled(): boolean {
      return this.odom_format === 'DMS'
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
</style>
