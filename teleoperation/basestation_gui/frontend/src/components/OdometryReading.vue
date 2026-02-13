<template>
  <div class="odom-container flex flex-row py-2 overflow-hidden items-stretch">
    <!-- Rover -->
    <div class="flex-1 flex flex-col px-4 relative divider-end">
      <div class="odom-section-header uppercase text-muted font-bold text-center mb-2 opacity-75">Rover</div>
      <div class="flex flex-col gap-1">
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Lat</span>
          <span class="odom-value font-mono font-bold">{{ rover_latitude_deg.toFixed(6) }}</span>
        </div>
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Lon</span>
          <span class="odom-value font-mono font-bold">{{ rover_longitude_deg.toFixed(6) }}</span>
        </div>
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Alt</span>
          <div class="flex items-baseline">
            <span class="odom-value font-mono font-bold">{{ rover_altitude.toFixed(1) }}</span>
            <span class="odom-unit font-mono text-muted ml-1">m</span>
          </div>
        </div>
      </div>
    </div>

    <!-- Base -->
    <div class="flex-1 flex flex-col px-4 relative divider-end">
      <div class="odom-section-header uppercase text-muted font-bold text-center mb-2 opacity-75">Base</div>
      <div class="flex flex-col gap-1">
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Lat</span>
          <span class="odom-value font-mono font-bold">{{ basestation_latitude_deg.toFixed(6) }}</span>
        </div>
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Lon</span>
          <span class="odom-value font-mono font-bold">{{ basestation_longitude_deg.toFixed(6) }}</span>
        </div>
      </div>
    </div>

    <!-- Orientation -->
    <div class="flex-1 flex flex-col px-4 relative divider-end">
      <div class="odom-section-header uppercase text-muted font-bold text-center mb-2 opacity-75">Orientation</div>
      <div class="flex flex-col gap-1">
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Yaw</span>
          <div class="flex items-baseline">
            <span class="odom-value font-mono font-bold">{{ rover_bearing_deg.toFixed(0) }}</span>
            <span class="odom-unit font-mono text-muted ml-1">&deg;</span>
          </div>
        </div>
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Pit</span>
          <div class="flex items-baseline">
            <span class="odom-value font-mono font-bold">{{ pitch.toFixed(0) }}</span>
            <span class="odom-unit font-mono text-muted ml-1">&deg;</span>
          </div>
        </div>
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Rol</span>
          <div class="flex items-baseline">
            <span class="odom-value font-mono font-bold">{{ roll.toFixed(0) }}</span>
            <span class="odom-unit font-mono text-muted ml-1">&deg;</span>
          </div>
        </div>
      </div>
    </div>

    <!-- Fix Status -->
    <div class="flex-1 flex flex-col px-4 relative divider-end">
      <div class="odom-section-header uppercase text-muted font-bold text-center mb-2 opacity-75">Fix</div>
      <div class="flex flex-col gap-1">
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Rov</span>
          <IndicatorDot :is-active="rover_status >= 0" :size="16" />
        </div>
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Drn</span>
          <IndicatorDot :is-active="drone_status >= 0" :size="16" />
        </div>
      </div>
    </div>

    <!-- IMU Cal -->
    <div class="flex-1 flex flex-col px-4 relative">
      <div class="odom-section-header uppercase text-muted font-bold text-center mb-2 opacity-75">IMU Cal</div>
      <div class="flex flex-col gap-1">
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Mag</span>
          <IndicatorDot :is-active="mag_calibration == 3" :size="16" />
        </div>
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Gyr</span>
          <IndicatorDot :is-active="gyro_calibration == 3" :size="16" />
        </div>
        <div class="odom-row flex items-center justify-between gap-2 whitespace-nowrap leading-none">
          <span class="odom-label font-mono uppercase text-muted">Acc</span>
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
