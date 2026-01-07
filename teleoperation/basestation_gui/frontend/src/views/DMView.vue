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
        <div v-if="item.i === 'rover-3d'" class="island rounded overflow-hidden h-100">
          <Rover3D class="w-100 h-100" />
        </div>

        <div v-else-if="item.i === 'odometry'" class="island p-2 rounded h-100">
          <OdometryReading />
        </div>

        <div v-else-if="item.i === 'moteus'" class="island p-2 rounded d-flex flex-row gap-2 h-100">
          <ControllerDataTable mode="arm" header="Arm" />
          <ControllerDataTable mode="drive" header="Drive" />
        </div>

        <div v-else-if="item.i === 'controls'" class="island p-2 rounded d-flex flex-row justify-content-between h-100">
          <ArmControls />
          <GimbalControls />
          <DriveControls />
        </div>

        <div v-else-if="item.i === 'map'" class="island p-0 rounded overflow-hidden h-100">
          <BasicMap />
        </div>

        <div v-else-if="item.i === 'waypoints'" class="island p-2 rounded h-100">
          <BasicWaypointEditor :enableDrone="true" />
        </div>
      </GridItem>
    </GridLayout>
  </div>
</template>

<script lang="ts" setup>
import { onMounted, onUnmounted } from 'vue'
import { GridLayout, GridItem } from 'vue-grid-layout-v3'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import ArmControls from '@/components/ArmControls.vue'
import BasicMap from '@/components/BasicRoverMap.vue'
import BasicWaypointEditor from '@/components/BasicWaypointEditor.vue'
import OdometryReading from '@/components/OdometryReading.vue'
import DriveControls from '@/components/DriveControls.vue'
import GimbalControls from '@/components/GimbalControls.vue'
import Rover3D from '@/components/Rover3D.vue'
import { useWebsocketStore } from '@/stores/websocket'
import { useGridLayout } from '@/composables/useGridLayout'

const defaultLayout = [
  { x: 0, y: 0, w: 6, h: 4, i: 'rover-3d' },
  { x: 0, y: 4, w: 6, h: 2, i: 'odometry' },
  { x: 0, y: 6, w: 6, h: 2, i: 'controls' },
  { x: 0, y: 8, w: 6, h: 4, i: 'moteus' },
  { x: 6, y: 0, w: 6, h: 6, i: 'map' },
  { x: 6, y: 6, w: 6, h: 6, i: 'waypoints' },
]

const { wrapperRef, rowHeight, layout, locked, saveLayout } = useGridLayout(
  'dmView_gridLayout',
  defaultLayout
)

const websocketStore = useWebsocketStore()

onMounted(() => {
  websocketStore.setupWebSocket('arm')
  websocketStore.setupWebSocket('drive')
  websocketStore.setupWebSocket('chassis')
  websocketStore.setupWebSocket('nav')
})

onUnmounted(() => {
  websocketStore.closeWebSocket('arm')
  websocketStore.closeWebSocket('drive')
  websocketStore.closeWebSocket('chassis')
  websocketStore.closeWebSocket('nav')
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
