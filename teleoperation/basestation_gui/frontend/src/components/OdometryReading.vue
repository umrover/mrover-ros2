<template>
  <div class="odom-wrap">
    <div class="odom">
      <p>Current odometry reading:</p>
      <div>
        <p>{{ formatted_odom.lat.d }}º</p>
        <p v-if="min_enabled">{{ formatted_odom.lat.m }}' N</p>
        <p v-if="sec_enabled">{{ formatted_odom.lat.s }}" N</p>
      </div>
      <div>
        <p>{{ formatted_odom.lon.d }}º</p>
        <p v-if="min_enabled">{{ formatted_odom.lon.m }}' E</p>
        <p v-if="sec_enabled">{{ formatted_odom.lon.s }}" E</p>
      </div>
      <p>Bearing: {{ rover_bearing_deg.toFixed(2) }}º</p>
      <p>A: {{ rover_altitude.toFixed(2) }}m</p>
      <p>Odom Status: {{ get_odom_status }}</p>
      <p>Drone Status: {{ get_drone_status }}</p>
    </div>
    <div class="calibration imu">
      <IMUCalibration></IMUCalibration>
    </div>
    <div class="flightindicator">
      <FlightAttitudeIndicator></FlightAttitudeIndicator>
    </div>
    <div class="basestation-odom">
      <p>Basestation Coordinates:</p>
      <div>
        <p>{{ formatted_basestation_odom.lat.d }}º</p>
        <p v-if="min_enabled">{{ formatted_basestation_odom.lat.m }}' N</p>
        <p v-if="sec_enabled">{{ formatted_basestation_odom.lat.s }}" N</p>
      </div>
      <div>
        <p>{{ formatted_basestation_odom.lon.d }}º</p>
        <p v-if="min_enabled">{{ formatted_basestation_odom.lon.m }}' E</p>
        <p v-if="sec_enabled">{{ formatted_basestation_odom.lon.s }}" E</p>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { convertDMS, quaternionToMapAngle } from '../utils/map.js'
import Vuex from 'vuex'
const { mapGetters, mapState } = Vuex
import IMUCalibration from './IMUCalibration.vue'
import FlightAttitudeIndicator from './FlightAttitudeIndicator.vue'
import type { Odom, FormattedOdom } from '../types/coordinates'
import type { WebSocketState } from '../types/websocket.js'

export default {
  components: {
    FlightAttitudeIndicator,
    IMUCalibration,
  },

  emits: ['odom', 'drone_odom', 'basestation_odom'],

  data() {
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
      navMessage: (state: WebSocketState) => state.messages['nav']
    }),

    ...mapGetters('map', {
      odom_format: 'odomFormat',
    }),
    formatted_odom(): FormattedOdom {
      return {
        lat: convertDMS(
          { d: this.rover_latitude_deg, m: 0, s: 0 },
          this.odom_format as string,
        ),
        lon: convertDMS(
          { d: this.rover_longitude_deg, m: 0, s: 0 },
          this.odom_format as string,
        ),
      }
    },
    formatted_basestation_odom(): FormattedOdom {
      return {
        lat: convertDMS(
          { d: this.basestation_latitude_deg, m: 0, s: 0 },
          this.odom_format as string,
        ),
        lon: convertDMS(
          { d: this.basestation_longitude_deg, m: 0, s: 0 },
          this.odom_format as string,
        ),
      }
    },
    min_enabled: function () {
      return this.odom_format != 'D'
    },
    sec_enabled: function () {
      return this.odom_format == 'DMS'
    },
    alt_available: function () {
      return !isNaN(this.rover_altitude)
    },
    get_odom_status: function () {
      if (this.rover_status) {
        return 'fixed'
      } else {
        return 'not fixed'
      }
    },
    get_drone_status: function () {
      if (this.drone_status) {
        return 'fixed'
      } else {
        return 'not fixed'
      }
    },
  },

  watch: {
    navMessage(msg) {
      if (msg.type == 'gps_fix') {
        this.rover_latitude_deg = msg.latitude
        this.rover_longitude_deg = msg.longitude
        this.rover_altitude = msg.altitude
        this.rover_status = msg.status
        this.$emit('odom', {
          latitude_deg: this.rover_latitude_deg,
          longitude_deg: this.rover_longitude_deg,
          bearing_deg: this.rover_bearing_deg,
        } as Odom)
      } else if (msg.type == 'basestation_position') {
        console.log('basestation position received')
        this.basestation_latitude_deg = msg.latitude
        this.basestation_longitude_deg = msg.longitude
        this.basestation_status = msg.status
        this.$emit('basestation_odom', {
          latitude_deg: this.basestation_latitude_deg,
          longitude_deg: this.basestation_longitude_deg,
        } as Odom)
      } else if (msg.type == 'drone_waypoint') {
        this.drone_latitude_deg = msg.latitude
        this.drone_longitude_deg = msg.longitude
        this.drone_status = msg.status
        this.$emit('drone_odom', {
          latitude_deg: this.drone_latitude_deg,
          longitude_deg: this.drone_longitude_deg,
        })
      } else if (msg.type == 'orientation') { // currently inactive, DEPRECATED?
        this.rover_bearing_deg = quaternionToMapAngle(msg.orientation)
        this.$emit('odom', {
          latitude_deg: this.rover_latitude_deg,
          longitude_deg: this.rover_longitude_deg,
          bearing_deg: this.rover_bearing_deg,
        } as Odom)
      }
    },
  },
}
</script>

<style scoped>
.odom-wrap {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: auto auto;
  grid-template-rows: auto auto;
  grid-template-areas:
    'odom basestation-odom flightIndicator'
    'imu imu flightIndicator';
  height: auto;
  width: auto;
}

.odom {
  grid-area: odom;
}

.flightIndicator {
  grid-area: flightIndicator;
}

.basestation-odom {
  grid-area: basestation-odom;
}

.imu {
  grid-area: imu;
}

p {
  margin: 0px;
}
</style>
