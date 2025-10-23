<template>
  <div class="wrapper view-wrapper">
    <div class="data island p-2 rounded">
      <div :class="['rounded p-2 mb-2', ledColor]">
        <h2 class="text-center">Nav State: {{ navState }}</h2>
        <OdometryReading
          @odom="updateOdom"
          @basestation_odom="updateBasestationOdom"
        />
      </div>
      <div>
        <div
          v-if="!stuck_status"
          class="island p-2 rounded bg-success text-center"
        >
          <h4 class="m-0 p-0">Nominal Conditions</h4>
        </div>
        <div v-else class="island p-2 rounded bg-danger text-center">
          <h4 class="m-0 p-0">Obstruction Detected</h4>
        </div>
      </div>
    </div>
    <div
      class="feed island p-0 rounded position-relative ratio ratio-16x9 bg-black overflow-hidden"
    >
      <CameraFeed
        v-if="cameraFeedEnabled"
        :mission="'ZED'"
        :id="0"
        :name="'ZED'"
        class="z-0"
      />
      <img
        v-else
        src="/stream_placeholder.svg"
        alt="Camera Disabled"
        class="img-fluid h-100"
      />
      <div
        class="controls position-absolute d-inline-flex align-items-center gap-2 top-0 end-0 m-2 p-1 bg-white rounded z-1"
        style="max-width: max-content; max-height: max-content"
      >
        <input
          type="checkbox"
          class="form-check-input p-0"
          style="width: 14px; height: 14px; vertical-align: middle"
          :checked="cameraFeedEnabled"
          @change="toggleFeed"
        />
        <p class="mb-0 text-body" style="font-size: 14px; line-height: 18px">
          Enable
        </p>
      </div>
    </div>
    <div class="island p-0 rounded map overflow-hidden">
      <AutonRoverMap :odom="odom" :basestation="basestationOdom" />
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
      <MastGimbalControls />
    </div>
    <div class="island p-2 rounded moteus d-flex flex-column gap-2">
      <ControllerDataTable
        msg-type="drive_left_state"
        header="Drive Left States"
        class="border border-2 rounded p-2"
      />
      <ControllerDataTable
        msg-type="drive_right_state"
        header="Drive Right States"
        class="border border-2 rounded p-2"
      />
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'
import AutonRoverMap from '../components/AutonRoverMap.vue'
import AutonWaypointEditor from '../components/AutonWaypointEditor.vue'
import CameraFeed from '../components/CameraFeed.vue'
import OdometryReading from '../components/OdometryReading.vue'
import DriveControls from '../components/DriveControls.vue'
import MastGimbalControls from '../components/MastGimbalControls.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import { useWebsocketStore } from '@/stores/websocket'
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'

interface Odom {
  latitude_deg: number
  longitude_deg: number
  bearing_deg: number
}

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const autonomyStore = useAutonomyStore()
const { autonEnabled, teleopEnabled } = storeToRefs(autonomyStore)

const odom = ref<Odom | null>(null)
const basestationOdom = ref<Odom | null>(null)
const teleopEnabledCheck = ref(false)
const ledColor = ref('bg-danger')
const stuck_status = ref(false)
const navState = ref('OffState')
const cameraFeedEnabled = ref(true)

const scienceMessage = computed(() => messages.value['science'])
const navMessage = computed(() => messages.value['nav'])

watch(scienceMessage, (msg) => {
  if (msg.type == 'led') {
    if (msg.red)
      ledColor.value = 'bg-danger' //red
    else if (msg.green)
      ledColor.value = 'blink' //blinking green
    else if (msg.blue) ledColor.value = 'bg-primary' //blue
  }
})

watch(navMessage, (msg) => {
  if (msg.type == 'nav_state') {
    navState.value = msg.state
  }
})

const toggleFeed = () => {
  cameraFeedEnabled.value = !cameraFeedEnabled.value
}

const updateOdom = (newOdom: Odom) => {
  odom.value = newOdom
}

const updateBasestationOdom = (newOdom: Odom) => {
  basestationOdom.value = newOdom
}

const topics = ['auton', 'drive', 'nav', 'science']

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
  grid-gap: 10px;
  width: 100%;
  height: 100%;
  grid-template-columns: 45% auto auto;
  grid-template-rows: 40% 15% 1fr;
  grid-template-areas:
    'feed map map'
    'feed moteus waypoints'
    'data moteus waypoints';
  font-family: sans-serif;
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

.feed {
  grid-area: feed;
}
</style>
