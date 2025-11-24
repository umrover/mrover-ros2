<template>
  <div class="odom-container d-flex gap-2 p-2">
    <div class="odom-card">
      <div class="odom-header">Rover</div>
      <div class="odom-grid">
        <span class="odom-label">Lat</span>
        <span class="odom-value">{{ rover_latitude_deg.toFixed(6) }}</span>
        <span class="odom-label">Lon</span>
        <span class="odom-value">{{ rover_longitude_deg.toFixed(6) }}</span>
        <span class="odom-label">Alt</span>
        <span class="odom-value">{{ rover_altitude.toFixed(1) }}m</span>
      </div>
    </div>

    <div class="odom-card">
      <div class="odom-header">Base</div>
      <div class="odom-grid">
        <span class="odom-label">Lat</span>
        <span class="odom-value">{{ basestation_latitude_deg.toFixed(6) }}</span>
        <span class="odom-label">Lon</span>
        <span class="odom-value">{{ basestation_longitude_deg.toFixed(6) }}</span>
      </div>
    </div>

    <div class="odom-card">
      <div class="odom-header">Orientation</div>
      <div class="odom-grid">
        <span class="odom-label">Yaw</span>
        <span class="odom-value">{{ rover_bearing_deg.toFixed(1) }}&deg;</span>
        <span class="odom-label">Pitch</span>
        <span class="odom-value">{{ pitch.toFixed(1) }}&deg;</span>
        <span class="odom-label">Roll</span>
        <span class="odom-value">{{ roll.toFixed(1) }}&deg;</span>
      </div>
    </div>

    <div class="odom-card">
      <div class="odom-header">Status</div>
      <div class="odom-grid">
        <span class="odom-label">GPS</span>
        <span class="odom-value" :class="rover_status ? 'text-success' : 'text-danger'">
          {{ rover_status ? 'Fixed' : 'No Fix' }}
        </span>
        <span class="odom-label">Drone</span>
        <span class="odom-value" :class="drone_status ? 'text-success' : 'text-danger'">
          {{ drone_status ? 'Fixed' : 'No Fix' }}
        </span>
      </div>
    </div>

    <div class="odom-card">
      <div class="odom-header">IMU Cal</div>
      <div class="d-flex gap-2 justify-content-center">
        <div class="imu-item">
          <span class="imu-label">Mag</span>
          <LEDIndicator :name="mag_calibration.toString()" :show_name="true" :connected="mag_calibration == 3" />
        </div>
        <div class="imu-item">
          <span class="imu-label">Gyro</span>
          <LEDIndicator :name="gyro_calibration.toString()" :show_name="true" :connected="gyro_calibration == 3" />
        </div>
        <div class="imu-item">
          <span class="imu-label">Acc</span>
          <LEDIndicator :name="accel_calibration.toString()" :show_name="true" :connected="accel_calibration == 3" />
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
import LEDIndicator from './LEDIndicator.vue'
import type { NavMessage, CalibrationMessage } from '../types/coordinates'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const rover_latitude_deg = ref(38.4071654)
const rover_longitude_deg = ref(-110.7923927)
const rover_bearing_deg = ref(0)
const rover_altitude = ref(0)
const rover_status = ref(false)
const drone_status = ref(false)
const basestation_latitude_deg = ref(38.4071654)
const basestation_longitude_deg = ref(-110.7923927)

const pitch = ref(0)
const roll = ref(0)

const mag_calibration = ref(0)
const gyro_calibration = ref(0)
const accel_calibration = ref(0)

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (!msg) return
  const navMsg = msg as NavMessage
  if (navMsg.type === 'gps_fix') {
    rover_latitude_deg.value = navMsg.latitude
    rover_longitude_deg.value = navMsg.longitude
    rover_altitude.value = navMsg.altitude
    rover_status.value = navMsg.status
  } else if (navMsg.type === 'basestation_position') {
    basestation_latitude_deg.value = navMsg.latitude
    basestation_longitude_deg.value = navMsg.longitude
  } else if (navMsg.type === 'drone_waypoint') {
    drone_status.value = navMsg.status
  } else if (navMsg.type === 'orientation') {
    rover_bearing_deg.value = quaternionToMapAngle(navMsg.orientation)
    const { x: qx, y: qy, z: qz, w: qw } = navMsg.orientation
    pitch.value = (Math.asin(2 * (qx * qz - qy * qw)) * 180) / Math.PI
    roll.value = (Math.atan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)) * 180) / Math.PI
  } else if (navMsg.type === 'calibration') {
    const calMsg = msg as CalibrationMessage
    mag_calibration.value = calMsg.magnetometer_calibration
    gyro_calibration.value = calMsg.gyroscope_calibration
    accel_calibration.value = calMsg.acceleration_calibration
  }
})
</script>

<style scoped>
.odom-container {
  font-family: system-ui, -apple-system, sans-serif;
}

.odom-card {
  background: var(--bs-light, #f8f9fa);
  border: 1px solid var(--bs-border-color, #dee2e6);
  border-radius: 6px;
  padding: 6px 10px;
}

.odom-header {
  font-size: 0.65rem;
  font-weight: 600;
  text-transform: uppercase;
  color: var(--bs-secondary, #6c757d);
  margin-bottom: 4px;
  text-align: center;
}

.odom-grid {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 2px 8px;
  align-items: center;
}

.odom-label {
  font-size: 0.7rem;
  color: var(--bs-secondary, #6c757d);
}

.odom-value {
  font-size: 0.7rem;
  font-weight: 600;
  font-family: 'SF Mono', Monaco, monospace;
  text-align: right;
}

.imu-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 2px;
}

.imu-label {
  font-size: 0.6rem;
  color: var(--bs-secondary, #6c757d);
}
</style>
