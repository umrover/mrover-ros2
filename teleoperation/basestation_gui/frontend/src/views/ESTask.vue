<template>
  <div class="view-wrapper wrapper-es">
    <div class="island p-2 rounded arm-controls">
      <ArmControls />
    </div>
    <div class="island rounded rover-3d overflow-hidden">
      <Rover3D class="w-100 h-100" />
    </div>
    <div class="island p-2 rounded controller_state d-flex flex-column gap-2">
      <ControllerDataTable msg-type="arm_state" header="Arm States" />
      <ControllerDataTable msg-type="drive_left_state" header="Drive Left" />
      <ControllerDataTable msg-type="drive_right_state" header="Drive Right" />
    </div>
    <div class="island p-2 rounded auton-typing">
      <AutonTyping />
    </div>
  </div>
</template>

<script lang='ts' setup>
import { onMounted, onUnmounted } from 'vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import ArmControls from '@/components/ArmControls.vue'
import Rover3D from '@/components/Rover3D.vue'
import AutonTyping from '@/components/AutonTyping.vue'
import { useWebsocketStore } from '@/stores/websocket'

const websocketStore = useWebsocketStore()

onMounted(() => {
  websocketStore.setupWebSocket('arm')
  websocketStore.setupWebSocket('drive')
})

onUnmounted(() => {
  websocketStore.closeWebSocket('arm')
  websocketStore.closeWebSocket('drive')
})
</script>

<style>
.wrapper-es {
  display: grid;
  gap: 10px;
  width: 100%;
  height: 100%;
  grid-template-columns: 25% auto; /* was formerly 400px auto */
  grid-template-rows: auto 21% auto;
  grid-template-areas:
    'arm-controls rover-3d'
    'auton-typing rover-3d'
    'controller_state rover-3d';
  font-family: sans-serif;
}

.auton-typing {
  grid-area: auton-typing;
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
