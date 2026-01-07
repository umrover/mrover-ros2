<template>
  <div ref="wrapperRef" class="view-wrapper">
    <GridLayout
      v-model:layout="layout"
      :col-num="12"
      :row-height="rowHeight"
      :margin="[10, 10]"
      :is-draggable="!locked"
      :is-resizable="!locked"
      :vertical-compact="true"
      :use-css-transforms="true"
      @layout-updated="saveLayout"
    >
      <GridItem
        v-for="item in layout"
        :key="item.i"
        :x="item.x"
        :y="item.y"
        :w="item.w"
        :h="item.h"
        :i="item.i"
      >
        <div v-if="item.i === 'controls'" class="island p-2 rounded d-flex flex-row gap-1 h-100">
          <SPArmControls class="border border-2 p-1 rounded flex-fill" />
          <div class="d-flex flex-column gap-1">
            <GimbalControls class="border border-2 p-1 rounded flex-fill" />
            <DriveControls class="border border-2 p-1 rounded" />
          </div>
          <div class="d-flex flex-column gap-1">
            <FunnelControls
              @selectSite="updateSite"
              class="border border-2 p-1 rounded flex-fill"
            />
            <PanoramaControls class="border border-2 p-1 rounded" />
          </div>
        </div>

        <div v-else-if="item.i === 'odometry'" class="island p-2 rounded h-100">
          <OdometryReading class="rounded border border-2 w-100 h-100" />
        </div>

        <div v-else-if="item.i === 'sensors'" class="island p-2 rounded h-100 d-flex flex-column">
          <SensorData />
        </div>

        <div v-else-if="item.i === 'moteus'" class="island p-2 rounded d-flex gap-2 h-100">
          <ControllerDataTable mode="drive" header="Drive" />
          <ControllerDataTable mode="sp" header="SP" />
        </div>

        <div v-else-if="item.i === 'map'" class="island p-0 rounded overflow-hidden h-100">
          <BasicMap />
        </div>

        <div v-else-if="item.i === 'waypoints'" class="island p-1 rounded h-100">
          <BasicWaypointEditor />
        </div>
      </GridItem>
    </GridLayout>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { GridLayout, GridItem } from 'vue-grid-layout-v3'
import SensorData from '../components/SensorData.vue'
import BasicMap from '../components/BasicRoverMap.vue'
import BasicWaypointEditor from '../components/BasicWaypointEditor.vue'
import DriveControls from '../components/DriveControls.vue'
import GimbalControls from '../components/GimbalControls.vue'
import OdometryReading from '../components/OdometryReading.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import FunnelControls from '../components/FunnelControls.vue'
import PanoramaControls from '../components/PanoramaControls.vue'
import SPArmControls from '@/components/SPArmControls.vue'
import { scienceAPI } from '@/utils/api'
import { useWebsocketStore } from '@/stores/websocket'
import { useGridLayout } from '@/composables/useGridLayout'

const defaultLayout = [
  { x: 0, y: 0, w: 7, h: 3, i: 'controls' },
  { x: 0, y: 3, w: 7, h: 2, i: 'odometry' },
  { x: 0, y: 5, w: 7, h: 4, i: 'sensors' },
  { x: 0, y: 9, w: 7, h: 3, i: 'moteus' },
  { x: 7, y: 0, w: 5, h: 6, i: 'map' },
  { x: 7, y: 6, w: 5, h: 6, i: 'waypoints' },
]

const { wrapperRef, rowHeight, layout, locked, saveLayout } = useGridLayout(
  'scienceView_gridLayout',
  defaultLayout
)

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
    const radians = site_to_radians[siteSelect.value]
    if (radians !== undefined) {
      await scienceAPI.setGearDiffPosition(radians, false)
    }
  } catch (error) {
    console.error('Failed to set gear differential position:', error)
  }
}

onMounted(() => {
  websocketStore.setupWebSocket('arm')
  websocketStore.setupWebSocket('chassis')
  websocketStore.setupWebSocket('drive')
  websocketStore.setupWebSocket('nav')
  websocketStore.setupWebSocket('science')
})

onUnmounted(() => {
  websocketStore.closeWebSocket('arm')
  websocketStore.closeWebSocket('chassis')
  websocketStore.closeWebSocket('drive')
  websocketStore.closeWebSocket('nav')
  websocketStore.closeWebSocket('science')
})
</script>

<style>
.vue-grid-layout {
  height: 100% !important;
  width: 100% !important;
}

.vue-grid-item {
  touch-action: none;
}

.vue-grid-item > div {
  height: 100%;
  width: 100%;
}
</style>
