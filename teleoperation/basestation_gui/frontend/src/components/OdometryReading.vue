<template>
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
        </div>
      </div>
    </div>

    <!-- Base -->
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
        </div>
      </div>
    </div>

    <!-- Orientation -->
    <div class="odom-col">
      <span class="odom-header">Orient.</span>
      <div class="odom-rows">
        <div class="odom-row">
          <span class="odom-label">Yaw</span>
          <span class="odom-value" :class="{ 'fmt-num-no-data': rover_bearing_deg == null }">{{ fmtDeg(rover_bearing_deg) }}<span class="odom-unit">&deg;</span></span>
        </div>
        <div class="odom-row">
          <span class="odom-label">Pit</span>
          <span class="odom-value" :class="{ 'fmt-num-no-data': pitch == null }">{{ fmtDeg(pitch) }}<span class="odom-unit">&deg;</span></span>
        </div>
        <div class="odom-row">
          <span class="odom-label">Rol</span>
          <span class="odom-value" :class="{ 'fmt-num-no-data': roll == null }">{{ fmtDeg(roll) }}<span class="odom-unit">&deg;</span></span>
        </div>
      </div>
    </div>

    <!-- Fix Status -->
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
        </div>
      </div>
    </div>

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


function fmtDeg(value: number | null): string {
  if (value === null) return '\u2012\u2012\u2012'
  return value.toFixed(0)
}

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
.odom-grid {
  display: grid;
  grid-template-columns: 3fr 3fr 2fr 2fr 2fr 2fr;
  height: 100%;
  font-family: var(--cmd-font-mono);
  font-size: 1.1rem;
}

.odom-col {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  padding: 0.5rem 0.75rem;
  border-right: 1px solid var(--cmd-panel-border);
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

</style>
