<template>
<<<<<<< HEAD
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
=======
  <div class="odom-grid">
    <!-- Rover -->
    <div class="odom-col">
      <span class="odom-header">Rover</span>
      <div class="odom-rows">
        <div class="odom-coord-row">
          <span class="odom-value" v-html="formatNumber(rover_latitude_deg, 3, 6)"></span>
          <span class="odom-unit">{{ dirLabel(rover_latitude_deg, 'N', 'S') }}</span>
        </div>
        <div class="odom-coord-row">
          <span class="odom-value" v-html="formatNumber(rover_longitude_deg, 3, 6)"></span>
          <span class="odom-unit">{{ dirLabel(rover_longitude_deg, 'E', 'W') }}</span>
        </div>
        <div class="odom-coord-row">
          <span class="odom-value" v-html="formatNumber(rover_altitude, 3, 1)"></span>
          <span class="odom-unit">m</span>
>>>>>>> origin/main
        </div>
      </div>
    </div>

    <!-- Base -->
<<<<<<< HEAD
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
=======
    <div class="odom-col">
      <span class="odom-header">Base</span>
      <div class="odom-rows">
        <div class="odom-coord-row">
          <span class="odom-value" v-html="formatNumber(basestation_latitude_deg, 3, 6)"></span>
          <span class="odom-unit">{{ dirLabel(basestation_latitude_deg, 'N', 'S') }}</span>
        </div>
        <div class="odom-coord-row">
          <span class="odom-value" v-html="formatNumber(basestation_longitude_deg, 3, 6)"></span>
          <span class="odom-unit">{{ dirLabel(basestation_longitude_deg, 'E', 'W') }}</span>
>>>>>>> origin/main
        </div>
      </div>
    </div>

    <!-- Orientation -->
<<<<<<< HEAD
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
=======
    <div class="odom-col">
      <span class="odom-header">Orient.</span>
      <div class="odom-rows">
        <div class="odom-row">
          <span class="odom-label">Yaw</span>
          <span class="odom-value" v-html="formatNumber(rover_bearing_deg, 3, 0)"></span><span class="odom-unit">&deg;</span>
        </div>
        <div class="odom-row">
          <span class="odom-label">Pit</span>
          <span class="odom-value" v-html="formatNumber(pitch, 3, 0)"></span><span class="odom-unit">&deg;</span>
        </div>
        <div class="odom-row">
          <span class="odom-label">Rol</span>
          <span class="odom-value" v-html="formatNumber(roll, 3, 0)"></span><span class="odom-unit">&deg;</span>
>>>>>>> origin/main
        </div>
      </div>
    </div>

    <!-- Fix Status -->
<<<<<<< HEAD
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
=======
    <div class="odom-col">
      <span class="odom-header">Fix</span>
      <div class="odom-rows">
        <div class="odom-row">
          <span class="odom-label">Rov</span>
          <IndicatorDot :is-active="rover_status != null && rover_status >= 0" :size="16" />
        </div>
        <div class="odom-row">
          <span class="odom-label">Drn</span>
          <IndicatorDot :is-active="drone_status != null && drone_status >= 0" :size="16" />
>>>>>>> origin/main
        </div>
      </div>
    </div>

<<<<<<< HEAD
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
=======
    <!-- IMU -->
    <div class="odom-col">
      <span class="odom-header">IMU</span>
      <div class="odom-rows">
        <div class="odom-row">
          <span class="odom-label">Mag</span>
          <IndicatorDot :is-active="mag_calibration === 3" :size="16" />
        </div>
        <div class="odom-row">
          <span class="odom-label">Gyr</span>
          <IndicatorDot :is-active="gyro_calibration === 3" :size="16" />
        </div>
        <div class="odom-row">
          <span class="odom-label">Acc</span>
          <IndicatorDot :is-active="accel_calibration === 3" :size="16" />
        </div>
      </div>
    </div>

    <!-- Velocity -->
    <div class="odom-col odom-col-last">
      <span class="odom-header">VEL</span>
      <div class="odom-rows">
        <div class="odom-coord-row">
          <span class="odom-value" v-html="formatNumber(linear_x, 2, 2)"></span>
          <span class="odom-unit">m/s</span>
        </div>
        <div class="odom-coord-row">
          <span class="odom-value" v-html="formatNumber(angular_z, 2, 2)"></span>
          <span class="odom-unit">&deg;/s</span>
>>>>>>> origin/main
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { quaternionToMapAngle } from '../utils/map.ts'
import { formatNumber } from '@/utils/formatNumber'
import type { GpsFixMessage, BasestationPositionMessage, DroneWaypointMessage, OrientationMessage, CalibrationMessage } from '../types/coordinates'
import type { CmdVelMessage } from '@/types/websocket'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()

const rover_latitude_deg = ref<number | null>(null)
const rover_longitude_deg = ref<number | null>(null)
const rover_bearing_deg = ref<number | null>(null)
const rover_altitude = ref<number | null>(null)
const rover_status = ref<number | null>(null)
const drone_status = ref<number | null>(null)
const basestation_latitude_deg = ref<number | null>(null)
const basestation_longitude_deg = ref<number | null>(null)

const pitch = ref<number | null>(null)
const roll = ref<number | null>(null)

const mag_calibration = ref<number | null>(null)
const gyro_calibration = ref<number | null>(null)
const accel_calibration = ref<number | null>(null)

const linear_x = ref<number | null>(null)
const angular_z = ref<number | null>(null)


function dirLabel(value: number | null, pos: string, neg: string): string {
  if (value === null) return '-'
  return value >= 0 ? pos : neg
}

websocketStore.onMessage<GpsFixMessage>('nav', 'gps_fix', msg => {
  rover_latitude_deg.value = msg.latitude
  rover_longitude_deg.value = msg.longitude
  rover_altitude.value = msg.altitude
  rover_status.value = msg.status.status
})

websocketStore.onMessage<BasestationPositionMessage>('nav', 'basestation_position', msg => {
  basestation_latitude_deg.value = msg.latitude
  basestation_longitude_deg.value = msg.longitude
})

websocketStore.onMessage<DroneWaypointMessage>('nav', 'drone_waypoint', msg => {
  drone_status.value = msg.status.status
})

websocketStore.onMessage<OrientationMessage>('nav', 'orientation', msg => {
  rover_bearing_deg.value = quaternionToMapAngle(msg.orientation)
  const { x: qx, y: qy, z: qz, w: qw } = msg.orientation
  pitch.value = (Math.asin(2 * (qx * qz - qy * qw)) * 180) / Math.PI
  roll.value =
    (Math.atan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)) * 180) /
    Math.PI
})

websocketStore.onMessage<CalibrationMessage>('nav', 'calibration', msg => {
  mag_calibration.value = msg.magnetometer_calibration
  gyro_calibration.value = msg.gyroscope_calibration
  accel_calibration.value = msg.acceleration_calibration
})

websocketStore.onMessage<CmdVelMessage>('nav', 'cmd_vel', msg => {
  linear_x.value = msg.linear.x
  angular_z.value = msg.angular.z * (180 / Math.PI)
})
</script>

<style scoped>
<<<<<<< HEAD
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
=======
.odom-grid {
  display: grid;
  grid-template-columns: 3fr 3fr 2fr 2fr 2fr 2fr;
  height: 100%;
  font-size: 1.1rem;
}

.odom-col {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  padding: 0.5rem 0.75rem;
  border-right: 1px solid var(--panel-border);
  min-width: 0;
}

.odom-col-last {
  border-right: none;
}

.odom-header {
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.06em;
  opacity: 0.6;
}

.odom-rows {
  display: flex;
  flex-direction: column;
  gap: 0.375rem;
}

.odom-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 0.5rem;
  line-height: 1;
}

.odom-coord-row {
  display: flex;
  align-items: center;
  line-height: 1;
}

.odom-value {
  font-weight: 600;
  font-variant-numeric: tabular-nums;
}

.odom-label {
  color: var(--text-muted);
  text-transform: uppercase;
}

.odom-unit {
  color: var(--text-muted);
  text-align: right;
  margin-left: auto;
}

>>>>>>> origin/main
</style>
