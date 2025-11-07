<template>
  <div class="d-flex gap-2 w-100 h-100 overflow-hidden px-2 flex-grow-1">
    <div class="d-flex flex-row gap-2 flex-grow-1 min-h-0 align-items-center">
      <div class="border rounded p-1 bg-light d-flex flex-column odom-section">
        <div class="fw-semibold text-center text-uppercase odom-title text-secondary mb-1">Rover</div>
        <div class="d-flex flex-column odom-rows flex-grow-1 justify-content-center">
          <div class="d-flex justify-content-between">
            <span class="text-muted odom-label">Lat:</span>
            <span class="fw-semibold font-monospace odom-value">{{ formatted_odom.lat }}º</span>
          </div>
          <div class="d-flex justify-content-between">
            <span class="text-muted odom-label">Lon:</span>
            <span class="fw-semibold font-monospace odom-value">{{ formatted_odom.lon }}º</span>
          </div>
          <div class="d-flex justify-content-between">
            <span class="text-muted odom-label">Bearing:</span>
            <span class="fw-semibold font-monospace odom-value">{{ rover_bearing_deg.toFixed(2) }}º</span>
          </div>
          <div class="d-flex justify-content-between">
            <span class="text-muted odom-label">Alt:</span>
            <span class="fw-semibold font-monospace odom-value">{{ rover_altitude.toFixed(2) }}m</span>
          </div>
        </div>
      </div>

      <div class="border rounded p-1 bg-light d-flex flex-column odom-section">
        <div class="fw-semibold text-center text-uppercase odom-title text-secondary mb-1">Base</div>
        <div class="d-flex flex-column odom-rows flex-grow-1 justify-content-center">
          <div class="d-flex justify-content-between">
            <span class="text-muted odom-label">Lat:</span>
            <span class="fw-semibold font-monospace odom-value">{{ formatted_basestation_odom.lat }}º</span>
          </div>
          <div class="d-flex justify-content-between">
            <span class="text-muted odom-label">Lon:</span>
            <span class="fw-semibold font-monospace odom-value">{{ formatted_basestation_odom.lon }}º</span>
          </div>
          <div class="d-flex justify-content-between">
            <span class="text-muted odom-label">GPS:</span>
            <span class="fw-semibold font-monospace odom-value">{{ get_odom_status }}</span>
          </div>
          <div class="d-flex justify-content-between">
            <span class="text-muted odom-label">Drone:</span>
            <span class="fw-semibold font-monospace odom-value">{{ get_drone_status }}</span>
          </div>
        </div>
      </div>
    </div>

    <div class="d-flex align-items-center justify-content-center h-100 attitude-container">
      <FlightAttitudeIndicator />
    </div>

    <div class="d-flex align-items-center imu-container">
      <div class="border rounded p-1 bg-light d-flex flex-column gap-1">
        <div class="fw-semibold text-center text-uppercase odom-title text-secondary">IMU Cal</div>
        <div class="d-flex flex-column gap-1">
          <div class="d-flex justify-content-between align-items-center">
            <span class="text-muted odom-label">Mag:</span>
            <LEDIndicator
              :name="mag_calibration.toString()"
              :show_name="true"
              :connected="mag_calibration == 3"
            />
          </div>
          <div class="d-flex justify-content-between align-items-center">
            <span class="text-muted odom-label">Gyro:</span>
            <LEDIndicator
              :name="gyro_calibration.toString()"
              :show_name="true"
              :connected="gyro_calibration == 3"
            />
          </div>
          <div class="d-flex justify-content-between align-items-center">
            <span class="text-muted odom-label">Accel:</span>
            <LEDIndicator
              :name="accel_calibration.toString()"
              :show_name="true"
              :connected="accel_calibration == 3"
            />
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import {quaternionToMapAngle } from '../utils/map.ts'
import FlightAttitudeIndicator from './FlightAttitudeIndicator.vue'
import LEDIndicator from './LEDIndicator.vue'
import type {
  FormattedOdom,
  NavMessage,
  CalibrationMessage,
} from '../types/coordinates'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const rover_latitude_deg = ref(38.4071654)
const rover_longitude_deg = ref(-110.7923927)
const rover_bearing_deg = ref(0)
const rover_altitude = ref(0)
const rover_status = ref(false)
const drone_latitude_deg = ref(38.4071654)
const drone_longitude_deg = ref(-110.7923927)
const drone_status = ref(false)
const basestation_latitude_deg = ref(38.4071654)
const basestation_longitude_deg = ref(-110.7923927)
const basestation_status = ref(false)

const mag_calibration = ref(0)
const gyro_calibration = ref(0)
const accel_calibration = ref(0)

const navMessage = computed(() => messages.value['nav'])

const formatted_odom = computed<FormattedOdom>(() => {
  return {
    lat: rover_latitude_deg.value,
    lon: rover_longitude_deg.value,
  }
})

const formatted_basestation_odom = computed<FormattedOdom>(() => {
  return {
    lat: basestation_latitude_deg.value,
    lon: basestation_longitude_deg.value,
  }
})



const get_odom_status = computed<string>(() => {
  return rover_status.value ? 'Fixed' : 'Not Fixed'
})

const get_drone_status = computed<string>(() => {
  return drone_status.value ? 'Fixed' : 'Not Fixed'
})

watch(navMessage, (msg) => {
  if (!msg) return
  const navMsg = msg as NavMessage;
  if (navMsg.type === 'gps_fix') {
    rover_latitude_deg.value = navMsg.latitude
    rover_longitude_deg.value = navMsg.longitude
    rover_altitude.value = navMsg.altitude
    rover_status.value = navMsg.status
  } else if (navMsg.type === 'basestation_position') {
    basestation_latitude_deg.value = navMsg.latitude
    basestation_longitude_deg.value = navMsg.longitude
    basestation_status.value = navMsg.status
  } else if (navMsg.type === 'drone_waypoint') {
    drone_latitude_deg.value = navMsg.latitude
    drone_longitude_deg.value = navMsg.longitude
    drone_status.value = navMsg.status
  } else if (navMsg.type === 'orientation') {
    rover_bearing_deg.value = quaternionToMapAngle(navMsg.orientation)
  } else if (navMsg.type === 'calibration') {
    const calMsg = msg as CalibrationMessage
    mag_calibration.value = calMsg.magnetometer_calibration
    gyro_calibration.value = calMsg.gyroscope_calibration
    accel_calibration.value = calMsg.acceleration_calibration
  }
})
</script>

<style scoped>
.odom-section {
  flex: 1 1 0;
  min-width: 0;
  min-height: 0;
  width: 0;
}

.odom-title {
  font-size: clamp(0.6rem, 1vw, 0.875rem);
  line-height: 1.2;
}

.odom-rows {
  gap: 0.25rem;
}

.odom-label {
  font-size: clamp(0.65rem, 1vw, 0.875rem);
  white-space: nowrap;
  line-height: 1.3;
}

.odom-value {
  font-size: clamp(0.65rem, 1vw, 0.875rem);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
  text-align: right;
  line-height: 1.3;
}

.attitude-container {
  flex: 1 1 auto;
  min-width: 120px;
  max-width: 200px;
  min-height: 0;
  height: 100%;
}

.attitude-container :deep(> div) {
  width: 100%;
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.attitude-container :deep(canvas) {
  max-width: 100%;
  max-height: 100%;
  width: auto !important;
  height: auto !important;
  object-fit: contain;
}

.imu-container {
  flex: 0 1 auto;
  min-width: 80px;
  max-width: 120px;
  min-height: 0;
}

.min-h-0 {
  min-height: 0;
}
</style>
