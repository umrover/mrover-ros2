<template>
  <div class="d-flex flex-column w-100 wrap">
    <div class="d-flex justify-content-between w-100 flex-wrap gap-1">
      <div class="d-flex flex-column gap-2">
        <div class="border border-2 border-secondary rounded p-2">
          <h5 class="m-0 p-0 text-center">Rover Odom</h5>
          <div class="d-flex gap-3 justify-content-center">
            <div class="odomModule">
              <small class="text-muted">Latitude</small>
              <p class="mb-0">{{ formatted_odom.lat }}º</p>
            </div>
            <div class="odomModule">
              <small class="text-muted">Longitude</small>
              <p class="mb-0">{{ formatted_odom.lon }}º</p>
            </div>
          </div>
        </div>
        <div class="d-flex justify-content-center gap-3">
          <div class="odomModule">
            <small class="text-muted">Bearing</small>
            <p class="mb-0">{{ rover_bearing_deg.toFixed(2) }}º</p>
          </div>
          <div class="odomModule">
            <small class="text-muted">Altitude</small>
            <p class="mb-0">{{ rover_altitude.toFixed(2) }} m</p>
          </div>
        </div>
      </div>
      <div class="d-flex justify-content-center align-items-center p-0 m-0">
        <FlightAttitudeIndicator />
      </div>
      <div class="d-flex flex-column gap-2">
        <div class="border border-2 border-secondary rounded p-2">
          <h5 class="m-0 p-0 font-monospace text-center">Basestation Odom</h5>
          <div class="d-flex gap-3 justify-content-center">
            <div class="odomModule">
              <small class="text-muted">Latitude</small>
              <p class="mb-0">{{ formatted_basestation_odom.lat }}º</p>
            </div>
            <div class="odomModule">
              <small class="text-muted">Longitude</small>
              <p class="mb-0">{{ formatted_basestation_odom.lon }}º</p>
            </div>
          </div>
        </div>
        <div class="d-flex justify-content-center gap-3">
          <div class="odomModule">
            <small class="text-muted">Odom Status</small>
            <p class="mb-0">{{ get_odom_status }}</p>
          </div>
          <div class="odomModule">
            <small class="text-muted">Drone Status</small>
            <p class="mb-0">{{ get_drone_status }}</p>
          </div>
        </div>
      </div>
    </div>
    <div class="d-flex justify-content-center">
      <IMUCalibration />
    </div>
  </div>
</template>

<script lang="ts" setup>
import { defineComponent, ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import {quaternionToMapAngle } from '../utils/map.ts'
import IMUCalibration from './IMUCalibration.vue'
import FlightAttitudeIndicator from './FlightAttitudeIndicator.vue'
import type {
  Odom,
  FormattedOdom,
  NavMessage,
} from '../types/coordinates.js'

const emit = defineEmits(['odom', 'drone_odom', 'basestation_odom'])

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

const alt_available = computed<boolean>(() => {
  return !isNaN(rover_altitude.value)
})

const get_odom_status = computed<string>(() => {
  return rover_status.value ? 'Fixed' : 'Not Fixed'
})

const get_drone_status = computed<string>(() => {
  return drone_status.value ? 'Fixed' : 'Not Fixed'
})

watch(navMessage, (msg: NavMessage | undefined) => {
  if (!msg) return

  if (msg.type === 'gps_fix') {
    rover_latitude_deg.value = msg.latitude
    rover_longitude_deg.value = msg.longitude
    rover_altitude.value = msg.altitude
    rover_status.value = msg.status
    emit('odom', {
      latitude_deg: rover_latitude_deg.value,
      longitude_deg: rover_longitude_deg.value,
      bearing_deg: rover_bearing_deg.value,
    } as Odom)
  } else if (msg.type === 'basestation_position') {
    basestation_latitude_deg.value = msg.latitude
    basestation_longitude_deg.value = msg.longitude
    basestation_status.value = msg.status
    emit('basestation_odom', {
      latitude_deg: basestation_latitude_deg.value,
      longitude_deg: basestation_longitude_deg.value,
    } as Odom)
  } else if (msg.type === 'drone_waypoint') {
    drone_latitude_deg.value = msg.latitude
    drone_longitude_deg.value = msg.longitude
    drone_status.value = msg.status
    emit('drone_odom', {
      latitude_deg: drone_latitude_deg.value,
      longitude_deg: drone_longitude_deg.value,
    })
  } else if (msg.type === 'orientation') {
    rover_bearing_deg.value = quaternionToMapAngle(msg.orientation)
    emit('odom', {
      latitude_deg: rover_latitude_deg.value,
      longitude_deg: rover_longitude_deg.value,
      bearing_deg: rover_bearing_deg.value,
    } as Odom)
  }
})
</script>

<style scoped>
.odomModule {
  width: 100px;
}

.wrap {
  min-width: 700px;
}
</style>
