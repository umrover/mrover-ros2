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
      <ControllerDataTable msg-type="drive_state" header="Drive States" />
    </div>
    <div>
      <AutonTyping />
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import ArmControls from '../components/ArmControls.vue'
import Rover3D from '../components/Rover3D.vue'
import AutonTyping from '../components/AutonTyping.vue'

export default defineComponent({
  components: {
    ControllerDataTable,
    ArmControls,
    Rover3D,
    AutonTyping,
  },

  mounted: function () {
    this.$store.dispatch('websocket/setupWebSocket', 'arm')
    this.$store.dispatch('websocket/setupWebSocket', 'drive')
  },

  unmounted: function () {
    this.$store.dispatch('websocket/closeWebSocket', 'arm')
    this.$store.dispatch('websocket/closeWebSocket', 'drive')
  },
})
</script>

<style>
.wrapper-es {
  display: grid;
  gap: 10px;
  width: 100%;
  height: 100%;
  grid-template-columns: 400px auto;
  grid-template-rows: 50% auto;
  grid-template-areas:
    'arm-controls rover-3d'
    'controller_state rover-3d';
  font-family: sans-serif;
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
