<template>
  <div class="view-wrapper d-flex gap-2 w-100 h-100">
    <div class="d-flex flex-column gap-2 flex-fill">
      <div class="island p-2 rounded d-flex gap-2">
        <SPArmControls class="border border-2 rounded flex-fill" />
        <MastGimbalControls class="border border-2 rounded flex-fill" />
        <FunnelControls
          @selectSite="updateSite"
          class="border border-2 rounded flex-fill"
        />
      </div>
      <div class="island p-2 rounded d-flex gap-2 align-items-center">
        <DriveControls class="border border-2 rounded" />
        <PanoControls class="border border-2 rounded flex-fill" />
      </div>
      <div class="island p-2 rounded">
        <OdometryReading class="rounded border border-2 w-100 h-100" />
      </div>
      <div class="island p-2 rounded">
        <SensorData :site="siteSelect" />
      </div>
      <div class="island p-2 rounded d-flex gap-2">
        <ControllerDataTable
          msg-type="drive_left_state"
          header="Drive Left"
          class="rounded border border-2 p-2 flex-fill"
        />
        <ControllerDataTable
          msg-type="drive_right_state"
          header="Drive Right"
          class="rounded border border-2 p-2 flex-fill"
        />
        <ControllerDataTable
          msg-type="sp_state"
          header="SP States"
          class="rounded border border-2 p-2 flex-fill"
        />
      </div>
    </div>
    <div class="d-flex flex-column gap-2 flex-fill">
      <div class="island p-0 rounded overflow-hidden flex-fill">
        <BasicMap />
      </div>
      <div class="island p-1 rounded min-h-50">
        <BasicWaypointEditor />
      </div>
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
import FunnelControls from '../components/FunnelControls.vue'
import PanoControls from '../components/PanoControls.vue'
import { scienceAPI } from '@/utils/api'
import { useWebsocketStore } from '@/stores/websocket'
import SPArmControls from '@/components/SPArmControls.vue'

const websocketStore = useWebsocketStore()

const siteSelect = ref(0)
const site_to_radians: { [key: number]: number } = {
  0: 0.0,
  1: Math.PI / 3,
  2: (2 * Math.PI) / 3,
  3: Math.PI,
  4: (4 * Math.PI) / 3,
  5: (5 * Math.PI) / 3,
}

const updateSite = async (selectedSite: number) => {
  siteSelect.value = selectedSite

  try {
    await scienceAPI.setGearDiffPosition(site_to_radians[siteSelect.value])
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
.min-h-50 {
  min-height: 50%;
}
</style>
