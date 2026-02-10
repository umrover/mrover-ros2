<template>
  <div class="odom-container d-flex flex-row py-2 overflow-hidden align-items-stretch">
    <!-- Rover -->
    <div class="flex-fill d-flex flex-column px-3 position-relative divider-end">
      <div class="odom-section-header text-uppercase text-muted fw-bold text-center mb-2 opacity-75">Rover</div>
      <div class="d-flex flex-column gap-1">
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Lat</span>
          <span class="odom-value font-monospace fw-bold">{{ rover_latitude_deg.toFixed(6) }}</span>
        </div>
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Lon</span>
          <span class="odom-value font-monospace fw-bold">{{ rover_longitude_deg.toFixed(6) }}</span>
        </div>
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Alt</span>
          <div class="d-flex align-items-baseline">
            <span class="odom-value font-monospace fw-bold">{{ rover_altitude.toFixed(1) }}</span>
            <span class="odom-unit font-monospace text-muted ms-1">m</span>
          </div>
        </div>
      </div>
    </div>

    <!-- Base -->
    <div class="flex-fill d-flex flex-column px-3 position-relative divider-end">
      <div class="odom-section-header text-uppercase text-muted fw-bold text-center mb-2 opacity-75">Base</div>
      <div class="d-flex flex-column gap-1">
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Lat</span>
          <span class="odom-value font-monospace fw-bold">{{ basestation_latitude_deg.toFixed(6) }}</span>
        </div>
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Lon</span>
          <span class="odom-value font-monospace fw-bold">{{ basestation_longitude_deg.toFixed(6) }}</span>
        </div>
      </div>
    </div>

    <!-- Orientation -->
    <div class="flex-fill d-flex flex-column px-3 position-relative divider-end">
      <div class="odom-section-header text-uppercase text-muted fw-bold text-center mb-2 opacity-75">Orientation</div>
      <div class="d-flex flex-column gap-1">
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Yaw</span>
          <div class="d-flex align-items-baseline">
            <span class="odom-value font-monospace fw-bold">{{ rover_bearing_deg.toFixed(0) }}</span>
            <span class="odom-unit font-monospace text-muted ms-1">&deg;</span>
          </div>
        </div>
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Pit</span>
          <div class="d-flex align-items-baseline">
            <span class="odom-value font-monospace fw-bold">{{ pitch.toFixed(0) }}</span>
            <span class="odom-unit font-monospace text-muted ms-1">&deg;</span>
          </div>
        </div>
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Rol</span>
          <div class="d-flex align-items-baseline">
            <span class="odom-value font-monospace fw-bold">{{ roll.toFixed(0) }}</span>
            <span class="odom-unit font-monospace text-muted ms-1">&deg;</span>
          </div>
        </div>
      </div>
    </div>

    <!-- Fix Status -->
    <div class="flex-fill d-flex flex-column px-3 position-relative divider-end">
      <div class="odom-section-header text-uppercase text-muted fw-bold text-center mb-2 opacity-75">Fix</div>
      <div class="d-flex flex-column gap-1">
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Rov</span>
          <IndicatorDot :is-active="rover_status >= 0" :size="16" />
        </div>
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Drn</span>
          <IndicatorDot :is-active="drone_status >= 0" :size="16" />
        </div>
      </div>
    </div>

    <!-- IMU Cal -->
    <div class="flex-fill d-flex flex-column px-3 position-relative">
      <div class="odom-section-header text-uppercase text-muted fw-bold text-center mb-2 opacity-75">IMU Cal</div>
      <div class="d-flex flex-column gap-1">
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Mag</span>
          <IndicatorDot :is-active="mag_calibration == 3" :size="16" />
        </div>
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Gyr</span>
          <IndicatorDot :is-active="gyro_calibration == 3" :size="16" />
        </div>
        <div class="odom-row d-flex align-items-center justify-content-between gap-2 text-nowrap lh-1">
          <span class="odom-label font-monospace text-uppercase text-muted">Acc</span>
          <IndicatorDot :is-active="accel_calibration == 3" :size="16" />
        </div>
      </div>
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
.odom-section-header {
  font-size: clamp(0.65rem, 0.9vw, 0.9rem);
  letter-spacing: 0.1em;
}

.odom-label {
  font-size: clamp(0.75rem, 1vw, 1rem);
  min-width: 2.5rem;
}

.odom-value {
  font-size: clamp(0.95rem, 1.3vw, 1.4rem);
}

.odom-unit {
  font-size: clamp(0.75rem, 1vw, 1rem);
}

.odom-row {
  height: 1.5rem;
}

.divider-end::after {
  content: '';
  position: absolute;
  right: 0;
  top: 15%;
  height: 70%;
  width: 1px;
  background: var(--cmd-panel-border);
}
</style>
