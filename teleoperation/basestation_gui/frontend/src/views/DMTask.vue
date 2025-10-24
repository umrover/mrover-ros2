<template>
  <div class="view-wrapper wrapper-dm">
    <div class='island p-2 rounded odom'>
      <OdometryReading @odom='updateOdom' @drone_odom="updateDroneOdom" />
    </div>
    <div class='island p-0 rounded map overflow-hidden'>
      <BasicMap :odom='odom' :drone_odom="drone_odom" />
    </div>
    <div class='island p-2 rounded waypoint-editor'>
      <BasicWaypointEditor :odom='odom' :droneWaypointButton='true'/>
    </div>
    <div class='island p-2 rounded arm-controls'>
      <ArmControls />
    </div>
    <div class='island rounded rover-3d overflow-hidden'>
      <Rover3D class="w-100 h-100"/>
    </div>
    <div class='island p-2 rounded controller_state d-flex flex-column gap-2'>
      <ControllerDataTable msg-type='arm_state' header='Arm States' />
      <ControllerDataTable msg-type='drive_state' header='Drive States' />
    </div>
    <div>
      <MastGimbalControls />
      <DriveControls />
    </div>
  </div>
</template>

<script lang='ts' setup>
import { ref, onMounted, onUnmounted } from 'vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import ArmControls from '@/components/ArmControls.vue'
import BasicMap from '@/components/BasicRoverMap.vue'
import BasicWaypointEditor from '@/components/BasicWaypointEditor.vue'
import OdometryReading from '@/components/OdometryReading.vue'
import DriveControls from '@/components/DriveControls.vue'
import MastGimbalControls from '@/components/MastGimbalControls.vue'
import Rover3D from '@/components/Rover3D.vue'
import { useWebsocketStore } from '@/stores/websocket'

import type { Odom } from '@/types/coordinates'

const websocketStore = useWebsocketStore()

const odom = ref<Odom>({ latitude_deg: 0, longitude_deg: 0, bearing_deg: 0 })
const drone_odom = ref<Odom>({ latitude_deg: 0, longitude_deg: 0 }) // Use Odom for drone_odom

const updateOdom = (newOdom: Odom) => {
  odom.value = newOdom;
}

const updateDroneOdom = (newOdom: Odom) => { // newOdom can be Odom now
  drone_odom.value = newOdom;
}

onMounted(() => {
  websocketStore.setupWebSocket('arm')
  websocketStore.setupWebSocket('drive')
  websocketStore.setupWebSocket('mast')
  websocketStore.setupWebSocket('nav')
})

onUnmounted(() => {
  websocketStore.closeWebSocket('arm')
  websocketStore.closeWebSocket('drive')
  websocketStore.closeWebSocket('mast')
  websocketStore.closeWebSocket('nav')
})
</script>

<style>
.wrapper-dm {
  display: grid;
  gap: 10px;
  width: 100%;
  height: 100%;
  grid-template-columns: 45% auto auto;
  grid-template-rows: 45% 1fr 1fr 1fr;
  grid-template-areas:
    'rover-3d map map'
    'rover-3d controller_state waypoint-editor'
    'arm-controls controller_state waypoint-editor'
    'odom controller_state waypoint-editor';
  font-family: sans-serif;
}

.map {
  grid-area: map;
}

.odom {
  grid-area: odom;
}

.waypoint-editor {
  grid-area: waypoint-editor;
}

.arm-controls {
  grid-area: arm-controls;
}

.controller_state {
  grid-area: controller_state;
}

.rover-3d {
  grid-area: rover-3d;
}
</style>
