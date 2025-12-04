<template>
  <div class="wrapper view-wrapper">
    <div class="data island p-2 rounded d-flex flex-column gap-2">
      <!-- Odometry Reading Box -->
      <div class="rounded p-2 border border-2">
        <OdometryReading />
      </div>

      <!-- Conditions Box -->
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

      <!-- Nav State Box -->
      <div :class="['rounded p-2 flex-fill d-flex align-items-center justify-content-center', ledColor]">
        <h3 class="m-0">Nav State: {{ navState }}</h3>
      </div>
    </div>
    <div class="island p-0 rounded map overflow-hidden">
      <AutonRoverMap />
    </div>
    <div class="island p-2 rounded waypoints">
      <AutonWaypointEditor @toggleTeleop="teleopEnabledCheck = $event" />
    </div>
    <!--Enable the drive controls if auton is off-->
    <div
      v-if="!autonEnabled && teleopEnabledCheck"
      v-show="false"
      class="driveControls"
    >
      <DriveControls />
      <GimbalControls />
    </div>
    <div class="island p-2 rounded moteus d-flex gap-2">
      <ControllerDataTable
        msg-type="drive_left_state"
        header="Drive Left States"
        class="border border-2 rounded p-2 flex-fill"
      />
      <ControllerDataTable
        msg-type="drive_right_state"
        header="Drive Right States"
        class="border border-2 rounded p-2 flex-fill"
      />
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'
import AutonRoverMap from '../components/AutonRoverMap.vue'
import AutonWaypointEditor from '../components/AutonWaypointEditor.vue'
import OdometryReading from '../components/OdometryReading.vue'
import DriveControls from '../components/DriveControls.vue'
import GimbalControls from '../components/GimbalControls.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import { useWebsocketStore } from '@/stores/websocket'
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'

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
      if (typedMsg.red)
        ledColor.value = 'bg-danger' //red
      else if (typedMsg.green)
        ledColor.value = 'blink' //blinking green
      else if (typedMsg.blue) ledColor.value = 'bg-primary' //blue
    }
  }
})

watch(navMessage, (msg: unknown) => {
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

<style scoped>
.wrapper {
  display: grid;
  gap: 0.625rem;
  width: 100%;
  height: 100%;
  max-width: 100vw;
  max-height: 100vh;
  overflow: hidden;
  box-sizing: border-box;
  grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
  grid-template-rows: minmax(200px, 40%) auto auto;
  grid-template-areas:
    'map map'
    'data waypoints'
    'moteus waypoints';
  font-family: sans-serif;
}

.wrapper > * {
  min-width: 0;
  min-height: 0;
  overflow: auto;
}

.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

.moteus {
  grid-area: moteus;
}

.data {
  grid-area: data;
}
</style>
