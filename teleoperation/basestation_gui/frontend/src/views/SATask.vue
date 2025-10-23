<template>
  <div class="wrapper view-wrapper overflow-x-hidden h-100">
    <div class="island p-2 rounded controls d-flex gap-2">
      
      <HexHub @selectSite="updateSite" @orientation="updateOrientation" class="border boder-2 rounded"/>
      
      <PanoCam class="border boder-2 rounded"/>
      <DriveControls />
      <MastGimbalControls />
    </div>
    <div class="island p-0 rounded map overflow-hidden">
      <BasicMap :odom="odom" />
    </div>
    <div class="island p-3 rounded waypoints">
      <BasicWaypointEditor :odom="odom" />
    </div>
    <div class="island p-1 rounded data d-flex gap-2">
      <OdometryReading @odom="updateOdom" class="rounded border border-2 p-2"/>
      <ControllerDataTable msg-type="drive_state" header="Drive States" class="rounded border border-2 p-2"/>
      <ControllerDataTable msg-type="sa_state" header="SA States" class="rounded border border-2 p-2"/>
      
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onUnmounted } from 'vue'
import BasicMap from '@/components/BasicRoverMap.vue'

import BasicWaypointEditor from '@/components/BasicWaypointEditor.vue'
import DriveControls from '@/components/DriveControls.vue'
import MastGimbalControls from '@/components/MastGimbalControls.vue'
import OdometryReading from '@/components/OdometryReading.vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'



import HexHub from '@/components/HexHub.vue'

import PanoCam from '@/components/PanoCam.vue'
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
  grid-template-columns: 55% auto; 
  grid-template-rows: auto 1fr auto;
  grid-template-areas:
    'controls controls'
    'map waypoints'
    'data data';
  font-family: sans-serif;
  height: auto;
  width: 100%;
}

.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

.data {
  grid-area: data;
}

.soilData {
  grid-area: soilData;
}

.controls {
  grid-area: controls;
}
</style>
