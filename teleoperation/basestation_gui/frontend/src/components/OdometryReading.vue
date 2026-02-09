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

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import { quaternionToMapAngle } from '../utils/map.ts'
import type { NavMessage, CalibrationMessage } from '../types/coordinates'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const rover_latitude_deg = ref(38.4071654)
const rover_longitude_deg = ref(-110.7923927)
const rover_bearing_deg = ref(0)
const rover_altitude = ref(0)
const rover_status = ref(-1)
const drone_status = ref(-1)
const basestation_latitude_deg = ref(38.4071654)
const basestation_longitude_deg = ref(-110.7923927)

const pitch = ref(0)
const roll = ref(0)

const mag_calibration = ref(0)
const gyro_calibration = ref(0)
const accel_calibration = ref(0)

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, msg => {
  if (!msg) return
  const navMsg = msg as NavMessage
  if (navMsg.type === 'gps_fix') {
    rover_latitude_deg.value = navMsg.latitude
    rover_longitude_deg.value = navMsg.longitude
    rover_altitude.value = navMsg.altitude
    rover_status.value = navMsg.status.status
  } else if (navMsg.type === 'basestation_position') {
    basestation_latitude_deg.value = navMsg.latitude
    basestation_longitude_deg.value = navMsg.longitude
  } else if (navMsg.type === 'drone_waypoint') {
    drone_status.value = navMsg.status.status
  } else if (navMsg.type === 'orientation') {
    rover_bearing_deg.value = quaternionToMapAngle(navMsg.orientation)
    const { x: qx, y: qy, z: qz, w: qw } = navMsg.orientation
    pitch.value = (Math.asin(2 * (qx * qz - qy * qw)) * 180) / Math.PI
    roll.value =
      (Math.atan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)) * 180) /
      Math.PI
  } else if (navMsg.type === 'calibration') {
    const calMsg = msg as CalibrationMessage
    mag_calibration.value = calMsg.magnetometer_calibration
    gyro_calibration.value = calMsg.gyroscope_calibration
    accel_calibration.value = calMsg.acceleration_calibration
  }
})
</script>

<style scoped>
.odom-section-header {
  font-size: clamp(0.65rem, 0.9vw, 0.9rem);
  letter-spacing: 0.1em;
}

.odom-label {
  min-width: 2.5rem;
  font-size: clamp(0.75rem, 1vw, 1rem);
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
  position: absolute;
  top: 15%;
  right: 0;
  width: 1px;
  height: 70%;
  content: '';
  background: var(--cmd-panel-border);
}
</style>
