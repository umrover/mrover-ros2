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
        <div
          v-if="item.i === 'data'"
          class="island p-2 rounded d-flex flex-column gap-2 h-100"
        >
          <div class="rounded p-2 border border-2">
            <OdometryReading />
          </div>
          <NavigationStatus />
        </div>

        <div
          v-else-if="item.i === 'map'"
          class="island p-0 rounded h-100 overflow-hidden"
        >
          <AutonRoverMap />
        </div>

        <div
          v-else-if="item.i === 'waypoints'"
          class="island p-2 rounded h-100 overflow-auto"
        >
          <AutonWaypointEditor @toggleTeleop="teleopEnabledCheck = $event" />
        </div>

        <div
          v-else-if="item.i === 'moteus'"
          class="island p-2 rounded h-100 overflow-hidden"
        >
          <ControllerDataTable mode="drive" header="Drive" />
        </div>
      </GridItem>
    </GridLayout>

    <div
      v-if="!autonEnabled && teleopEnabledCheck"
      v-show="false"
      class="driveControls"
    >
      <DriveControls />
      <GimbalControls />
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onUnmounted } from 'vue'
import { GridLayout, GridItem } from 'vue-grid-layout-v3'
import AutonRoverMap from '../components/AutonRoverMap.vue'
import AutonWaypointEditor from '../components/AutonWaypointEditor.vue'
import OdometryReading from '../components/OdometryReading.vue'
import NavigationStatus from '../components/NavigationStatus.vue'
import DriveControls from '../components/DriveControls.vue'
import GimbalControls from '../components/GimbalControls.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import { useWebsocketStore } from '@/stores/websocket'
import { useAutonomyStore } from '@/stores/autonomy'
import { useGridLayout } from '@/composables/useGridLayout'
import { storeToRefs } from 'pinia'

const defaultLayout = [
  { x: 0, y: 0, w: 6, h: 8, i: 'data' },
  { x: 0, y: 6, w: 6, h: 4, i: 'moteus' },
  { x: 6, y: 0, w: 6, h: 5, i: 'map' },
  { x: 6, y: 6, w: 6, h: 7, i: 'waypoints' },
]

const { wrapperRef, rowHeight, layout, locked, saveLayout } = useGridLayout(
  'autonView_gridLayout',
  defaultLayout
)

const websocketStore = useWebsocketStore()

const autonomyStore = useAutonomyStore()
const { autonEnabled } = storeToRefs(autonomyStore)

const teleopEnabledCheck = ref(false)

const topics = ['drive', 'nav', 'science', 'chassis']

onMounted(() => {
  for (const topic of topics) {
    websocketStore.setupWebSocket(topic)
  }
})

onUnmounted(() => {
  for (const topic of topics) {
    websocketStore.closeWebSocket(topic)
  }
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
