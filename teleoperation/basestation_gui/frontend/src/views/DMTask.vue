<template>
  <div class="view-wrapper d-grid w-100 h-100 gap-2 wrapper-dm">
    <div class="left-column d-flex flex-column gap-2">
      <div class='island rounded overflow-hidden flex-grow-1'>
        <Rover3D class="w-100 h-100"/>
      </div>
      <div class='island p-2 rounded'>
        <OdometryReading />
      </div>
      <div class='island p-2 rounded'>
        <ArmControls />
      </div>
      <div class='island p-2 rounded d-flex gap-2' style="flex: 0 0 auto;">
        <ControllerDataTable class="flex-fill" msg-type='arm_state' header='Arm States' />
        <ControllerDataTable class="flex-fill" msg-type='drive_left_state' header='Drive Left' />
        <ControllerDataTable class="flex-fill" msg-type='drive_right_state' header='Drive Right' />
      </div>
    </div>
    <div class="right-column d-flex flex-column gap-2">
      <div class='island p-0 rounded overflow-hidden' style="flex: 1; min-height: 0;">
        <BasicMap />
      </div>
      <div class='island p-2 rounded' style="flex: 1; min-height: 0;">
        <BasicWaypointEditor :enableDrone='true'/>
      </div>
    </div>
    <div>
      <MastGimbalControls />
      <DriveControls />
    </div>
  </div>
</template>

<script lang='ts' setup>
import { onMounted, onUnmounted } from 'vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import ArmControls from '@/components/ArmControls.vue'
import BasicMap from '@/components/BasicRoverMap.vue'
import BasicWaypointEditor from '@/components/BasicWaypointEditor.vue'
import OdometryReading from '@/components/OdometryReading.vue'
import DriveControls from '@/components/DriveControls.vue'
import MastGimbalControls from '@/components/MastGimbalControls.vue'
import Rover3D from '@/components/Rover3D.vue'
import { useWebsocketStore } from '@/stores/websocket'

const websocketStore = useWebsocketStore()

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
  grid-template-columns: 1fr 1fr;
}

.left-column,
.right-column {
  min-height: 0;
  min-width: 0;
}
</style>
