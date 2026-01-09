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
          <div>
            <div
              v-if="!stuck_status"
              class="island p-2 rounded bg-success text-center"
            >
              <h3 class="m-0 p-0">Nominal Conditions</h3>
            </div>
            <div v-else class="island p-2 rounded bg-danger text-center">
              <h3 class="m-0 p-0">Obstruction Detected</h3>
            </div>
          </div>
          <div
            :class="[
              'rounded p-2 flex-fill d-flex align-items-center justify-content-center',
              ledColor,
            ]"
          >
            <h3 class="m-0">Nav State: {{ navState }}</h3>
          </div>
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
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'
import { GridLayout, GridItem } from 'vue-grid-layout-v3'
import AutonRoverMap from '../components/AutonRoverMap.vue'
import AutonWaypointEditor from '../components/AutonWaypointEditor.vue'
import OdometryReading from '../components/OdometryReading.vue'
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
const { messages } = storeToRefs(websocketStore)

const autonomyStore = useAutonomyStore()
const { autonEnabled } = storeToRefs(autonomyStore)

const teleopEnabledCheck = ref(false)
const ledColor = ref('bg-danger')
const stuck_status = ref(false)
const navState = ref('OffState')

const scienceMessage = computed(() => messages.value['science'])
const navMessage = computed(() => messages.value['nav'])

watch(scienceMessage, (msg: unknown) => {
  if (typeof msg === 'object' && msg !== null && 'type' in msg) {
    const typedMsg = msg as {
      type: string
      red?: boolean
      green?: boolean
      blue?: boolean
    }
    if (typedMsg.type === 'led') {
      if (typedMsg.red) ledColor.value = 'bg-danger'
      else if (typedMsg.green) ledColor.value = 'blink'
      else if (typedMsg.blue) ledColor.value = 'bg-primary'
    }
  }
})

watch(navMessage, (msg: unknown) => {
  console.log("recv")
  if (typeof msg === 'object' && msg !== null && 'type' in msg) {
    const typedMsg = msg as { type: string; state?: string; color?: string }
    if (typedMsg.type === 'nav_state') {
      navState.value = typedMsg.state || 'OffState'
    } else if (typedMsg.type === 'led_color') {
      if (typedMsg.color === 'red') {
        ledColor.value = 'bg-danger'
      } else if (typedMsg.color === 'blinking-green') {
        ledColor.value = 'blink'
      } else if (typedMsg.color === 'blue') {
        ledColor.value = 'bg-primary'
      }
    }
  }
})

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
  ledColor.value = 'bg-white'
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

.blink {
  animation: blink-green 1s infinite;
}

@keyframes blink-green {
  0%, 50% {
    background-color: var(--bs-success);
  }
  51%, 100% {
    background-color: transparent;
  }
}
</style>
