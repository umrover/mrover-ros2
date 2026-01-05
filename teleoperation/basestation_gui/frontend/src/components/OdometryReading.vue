<template>
  <div
    class="d-flex align-items-start gap-2 text-nowrap overflow-hidden justify-content-between font-monospace"
  >
    <!-- Rover -->
    <div class="d-flex flex-column px-1 text-center flex-fill">
      <div class="fw-bold text-secondary text-uppercase mb-1">Rover</div>
      <div class="d-flex flex-column align-items-start">
        <span
          ><span class="text-muted me-1 d-inline-block" style="width: 2rem"
            >Lat</span
          ><span class="fw-bold">{{
            rover_latitude_deg.toFixed(6)
          }}</span></span
        >
        <span
          ><span class="text-muted me-1 d-inline-block" style="width: 2rem"
            >Lon</span
          ><span class="fw-bold">{{
            rover_longitude_deg.toFixed(6)
          }}</span></span
        >
        <span
          ><span class="text-muted me-1 d-inline-block" style="width: 2rem"
            >Alt</span
          ><span class="fw-bold">{{ rover_altitude.toFixed(1) }}m</span></span
        >
      </div>
    </div>

    <!-- Base -->
    <div class="d-flex flex-column px-1 text-center flex-fill">
      <div class="fw-bold text-secondary text-uppercase mb-1">Base</div>
      <div class="d-flex flex-column align-items-start">
        <span
          ><span class="text-muted me-1 d-inline-block" style="width: 2rem"
            >Lat</span
          ><span class="fw-bold">{{
            basestation_latitude_deg.toFixed(6)
          }}</span></span
        >
        <span
          ><span class="text-muted me-1 d-inline-block" style="width: 2rem"
            >Lon</span
          ><span class="fw-bold">{{
            basestation_longitude_deg.toFixed(6)
          }}</span></span
        >
      </div>
    </div>

    <!-- Orientation -->
    <div class="d-flex flex-column px-1 text-center flex-fill">
      <div class="fw-bold text-secondary text-uppercase mb-1">Orientation</div>
      <div class="d-flex flex-column gap-1">
        <span
          ><span class="text-muted me-1 d-inline-block" style="width: 3rem"
            >Yaw</span
          ><span class="fw-bold"
            >{{ rover_bearing_deg.toFixed(0) }}&deg;</span
          ></span
        >
        <span
          ><span class="text-muted me-1 d-inline-block" style="width: 3rem"
            >Pitch</span
          ><span class="fw-bold">{{ pitch.toFixed(0) }}&deg;</span></span
        >
        <span
          ><span class="text-muted me-1 d-inline-block" style="width: 3rem"
            >Roll</span
          ><span class="fw-bold">{{ roll.toFixed(0) }}&deg;</span></span
        >
      </div>
    </div>

    <!-- Fix Status -->
    <div class="d-flex flex-column px-1 text-center flex-fill">
      <div class="fw-bold text-secondary text-uppercase mb-1">Fix</div>
      <div class="d-flex flex-column gap-1">
        <div class="d-flex align-items-center justify-content-center gap-1">
          <span class="text-muted">Rov</span>
          <IndicatorDot :is-active="rover_status >= 0" />
        </div>
        <div class="d-flex align-items-center justify-content-center gap-1">
          <span class="text-muted">Drn</span>
          <IndicatorDot :is-active="drone_status >= 0" />
        </div>
      </div>
    </div>

    <!-- IMU Cal -->
    <div class="d-flex flex-column px-1 text-center flex-fill">
      <div class="fw-bold text-secondary text-uppercase mb-1">IMU Cal</div>
      <div class="d-flex flex-column gap-1">
        <div class="d-flex align-items-center justify-content-center gap-1">
          <span class="text-muted">MAG</span>
          <IndicatorDot :is-active="mag_calibration == 3" />
        </div>
        <div class="d-flex align-items-center justify-content-center gap-1">
          <span class="text-muted">GYR</span>
          <IndicatorDot :is-active="gyro_calibration == 3" />
        </div>
        <div class="d-flex align-items-center justify-content-center gap-1">
          <span class="text-muted">ACC</span>
          <IndicatorDot :is-active="accel_calibration == 3" />
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
