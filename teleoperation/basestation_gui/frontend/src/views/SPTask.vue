<template>
  <div class="wrapper view-wrapper">
    <div class="island p-0 rounded map overflow-hidden">
      <BasicMap :odom="odom" />
    </div>
    <div class="island p-2 rounded odom">
      <OdometryReading @odom="updateOdom" class="rounded border border-2 p-2" />
    </div>
    <div class="island p-2 rounded controller_state d-flex gap-2">
      <ControllerDataTable
        msg-type="drive_state"
        header="Drive States"
        class="rounded border border-2 p-2"
      />
      <ControllerDataTable
        msg-type="sa_state"
        header="SA States"
        class="rounded border border-2 p-2"
      />
    </div>
    <div class="island p-3 rounded waypoints">
      <BasicWaypointEditor :odom="odom" />
    </div>
    <div class="island p-2 rounded controls d-flex gap-2">
      <HexHub
        @selectSite="updateSite"
        @orientation="updateOrientation"
        class="border border-2 rounded"
      />
      <PanoCam class="border border-2 rounded" />
      <DriveControls />
      <MastGimbalControls />
    </div>
    <div class="island p-2 rounded sensors">
      <SensorData :site="siteSelect" />
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onUnmounted } from 'vue'
import SensorData from '../components/SensorData.vue'
import BasicMap from '../components/BasicRoverMap.vue'
import BasicWaypointEditor from '../components/BasicWaypointEditor.vue'
import DriveControls from '../components/DriveControls.vue'
import MastGimbalControls from '../components/MastGimbalControls.vue'
import OdometryReading from '../components/OdometryReading.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import HexHub from '../components/HexHub.vue'
import PanoCam from '../components/PanoCam.vue'
import { scienceAPI } from '@/utils/api'
import { useWebsocketStore } from '@/stores/websocket'

interface Odom {
  latitude_deg: number
  longitude_deg: number
  bearing_deg: number
}

const websocketStore = useWebsocketStore()

const odom = ref<Odom | null>(null)
const siteSelect = ref(0)
const orientation = ref(true)
const site_to_radians: { [key: number]: number } = {
  0: 0.0,
  1: (2 * Math.PI) / 5,
  2: (4 * Math.PI) / 5,
  3: (6 * Math.PI) / 5,
  4: (8 * Math.PI) / 5,
}

const updateOdom = (newOdom: Odom) => {
  odom.value = newOdom
}

const updateSite = async (selectedSite: number) => {
  siteSelect.value = selectedSite

  try {
    await scienceAPI.setGearDiffPosition(
      site_to_radians[siteSelect.value],
      orientation.value
    )
  } catch (error) {
    console.error('Failed to set gear differential position:', error)
  }
}

const updateOrientation = async (newOrientation: boolean) => {
  orientation.value = newOrientation

  try {
    await scienceAPI.setGearDiffPosition(
      site_to_radians[siteSelect.value],
      orientation.value
    )
  } catch (error) {
    console.error('Failed to set gear differential position:', error)
  }
}

onMounted(() => {
  websocketStore.setupWebSocket('arm')
  websocketStore.setupWebSocket('mast')
  websocketStore.setupWebSocket('nav')
  websocketStore.setupWebSocket('science')
})

onUnmounted(() => {
  websocketStore.closeWebSocket('arm')
  websocketStore.closeWebSocket('mast')
  websocketStore.closeWebSocket('nav')
  websocketStore.closeWebSocket('science')
})
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  width: 100%;
  height: 100%;
  grid-template-rows: 120px;
  grid-template-areas:
    'controls map map'
    'odom map map'
    'sensors sensors waypoints'
    'controller_state controller_state waypoints';
  font-family: sans-serif;
}

.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

.controls {
  grid-area: controls;
}

.sensors {
  grid-area: sensors;
}

.odom {
  grid-area: odom;
}

.controller_state {
  grid-area: controller_state;
}
</style>
