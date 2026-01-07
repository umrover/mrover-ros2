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
        <div v-if="item.i === 'arm-controls'" class="island p-2 rounded h-100">
          <ArmControls />
        </div>

        <div v-else-if="item.i === 'rover-3d'" class="island rounded overflow-hidden h-100">
          <Rover3D class="w-100 h-100" />
        </div>

        <div v-else-if="item.i === 'controller_state'" class="island p-2 rounded d-flex flex-column gap-2 h-100">
          <ControllerDataTable mode="arm" header="Arm" />
          <ControllerDataTable mode="drive" header="Drive" />
        </div>

        <div v-else-if="item.i === 'auton-typing'" class="island p-2 rounded h-100">
          <AutonTyping />
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
import Rover3D from '@/components/Rover3D.vue'
import AutonTyping from '@/components/AutonTyping.vue'
import { useWebsocketStore } from '@/stores/websocket'
import { useGridLayout } from '@/composables/useGridLayout'

const defaultLayout = [
  { x: 0, y: 0, w: 4, h: 2, i: 'arm-controls' },
  { x: 0, y: 2, w: 4, h: 3, i: 'auton-typing' },
  { x: 0, y: 5, w: 4, h: 7, i: 'controller_state' },
  { x: 4, y: 0, w: 8, h: 12, i: 'rover-3d' },
]

const { wrapperRef, rowHeight, layout, locked, saveLayout } = useGridLayout(
  'esView_gridLayout',
  defaultLayout
)

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
