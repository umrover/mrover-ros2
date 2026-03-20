<template>
  <div class="odom-grid">
    <!-- Rover -->
    <div class="odom-col">
      <span class="odom-header">Rover</span>
      <div class="odom-rows">
        <div class="odom-coord-row">
          <span class="odom-value"><span class="odom-pad">{{ padCoord(rover_latitude_deg, 3, 6).pad }}</span>{{ padCoord(rover_latitude_deg, 3, 6).num }}</span>
          <span class="odom-unit">{{ rover_latitude_deg >= 0 ? 'N' : 'S' }}</span>
        </div>
        <div class="odom-coord-row">
          <span class="odom-value"><span class="odom-pad">{{ padCoord(rover_longitude_deg, 3, 6).pad }}</span>{{ padCoord(rover_longitude_deg, 3, 6).num }}</span>
          <span class="odom-unit">{{ rover_longitude_deg >= 0 ? 'E' : 'W' }}</span>
        </div>
        <div class="odom-coord-row">
          <span class="odom-value"><span class="odom-pad">{{ padCoord(rover_altitude, 3, 1).pad }}</span>{{ padCoord(rover_altitude, 3, 1).num }}</span>
          <span class="odom-unit">m</span>
        </div>
      </div>
    </div>

    <!-- Base -->
    <div class="odom-col">
      <span class="odom-header">Base</span>
      <div class="odom-rows">
        <div class="odom-coord-row">
          <span class="odom-value"><span class="odom-pad">{{ padCoord(basestation_latitude_deg, 3, 6).pad }}</span>{{ padCoord(basestation_latitude_deg, 3, 6).num }}</span>
          <span class="odom-unit">{{ basestation_latitude_deg >= 0 ? 'N' : 'S' }}</span>
        </div>
        <div class="odom-coord-row">
          <span class="odom-value"><span class="odom-pad">{{ padCoord(basestation_longitude_deg, 3, 6).pad }}</span>{{ padCoord(basestation_longitude_deg, 3, 6).num }}</span>
          <span class="odom-unit">{{ basestation_longitude_deg >= 0 ? 'E' : 'W' }}</span>
        </div>
      </div>
    </div>

    <!-- Orientation -->
    <div class="odom-col">
      <span class="odom-header">Orient.</span>
      <div class="odom-rows">
        <div class="odom-row">
          <span class="odom-label">Yaw</span>
          <span class="odom-value">{{ rover_bearing_deg.toFixed(0) }}<span class="odom-unit">&deg;</span></span>
        </div>
        <div class="odom-row">
          <span class="odom-label">Pit</span>
          <span class="odom-value">{{ pitch.toFixed(0) }}<span class="odom-unit">&deg;</span></span>
        </div>
        <div class="odom-row">
          <span class="odom-label">Rol</span>
          <span class="odom-value">{{ roll.toFixed(0) }}<span class="odom-unit">&deg;</span></span>
        </div>
      </div>
    </div>

    <!-- Fix Status -->
    <div class="odom-col">
      <span class="odom-header">Fix</span>
      <div class="odom-rows">
        <div class="odom-row">
          <span class="odom-label">Rov</span>
          <IndicatorDot :is-active="rover_status >= 0" :size="16" />
        </div>
        <div class="odom-row">
          <span class="odom-label">Drn</span>
          <IndicatorDot :is-active="drone_status >= 0" :size="16" />
        </div>
      </div>
    </div>

    <!-- IMU Cal -->
    <div class="odom-col">
      <span class="odom-header">IMU</span>
      <div class="odom-rows">
        <div class="odom-row">
          <span class="odom-label">Mag</span>
          <IndicatorDot :is-active="mag_calibration == 3" :size="16" />
        </div>
        <div class="odom-row">
          <span class="odom-label">Gyr</span>
          <IndicatorDot :is-active="gyro_calibration == 3" :size="16" />
        </div>
        <div class="odom-row">
          <span class="odom-label">Acc</span>
          <IndicatorDot :is-active="accel_calibration == 3" :size="16" />
        </div>
      </div>
    </div>

    <!-- Velocity -->
    <div class="odom-col odom-col-last">
      <span class="odom-header">VEL</span>
      <div class="odom-rows">
        <div class="odom-coord-row">
          <span class="odom-value"><span class="odom-pad">{{ padCoord(linear_x, 2, 2).pad }}</span>{{ padCoord(linear_x, 2, 2).num }}</span>
          <span class="odom-unit">m/s</span>
        </div>
        <div class="odom-coord-row">
          <span class="odom-value"><span class="odom-pad">{{ padCoord(angular_z, 2, 2).pad }}</span>{{ padCoord(angular_z, 2, 2).num }}</span>
          <span class="odom-unit">rad/s</span>
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
import type { CmdVelMessage } from '@/types/websocket'
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

const linear_x = ref(0)
const angular_z = ref(0)

function padCoord(value: number, intDigits: number, fracDigits: number): { pad: string; num: string } {
  const abs = Math.abs(value)
  const fixed = abs.toFixed(fracDigits)
  const intPart = fixed.split('.')[0]
  const padCount = Math.max(0, intDigits - intPart.length)
  return { pad: '0'.repeat(padCount), num: fixed }
}

const navMessage = computed(() => messages.value['nav'] as (NavMessage | CmdVelMessage) | undefined)

watch(navMessage, msg => {
  if (!msg) return

  if (msg.type === 'cmd_vel') {
    linear_x.value = msg.linear.x
    angular_z.value = msg.angular.z
    return
  }

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
    const calMsg = navMsg as CalibrationMessage
    mag_calibration.value = calMsg.magnetometer_calibration
    gyro_calibration.value = calMsg.gyroscope_calibration
    accel_calibration.value = calMsg.acceleration_calibration
  }
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

.odom-pad {
  opacity: 0.25;
}
</style>
