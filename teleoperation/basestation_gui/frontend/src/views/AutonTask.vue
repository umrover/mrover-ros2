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

<script lang="ts">
import Vuex from 'vuex'
const { mapActions, mapGetters, mapState } = Vuex
import AutonRoverMap from '../components/AutonRoverMap.vue'
import AutonWaypointEditor from '../components/AutonWaypointEditor.vue'
import CameraFeed from '../components/CameraFeed.vue'
import OdometryReading from '../components/OdometryReading.vue'
import DriveControls from '../components/DriveControls.vue'
import MastGimbalControls from '../components/MastGimbalControls.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import { defineComponent } from 'vue'
import type { WebSocketState } from '../types/websocket'

interface Odom {
  latitude_deg: number
  longitude_deg: number
  bearing_deg: number
}

export default defineComponent({
  components: {
    ControllerDataTable,
    AutonRoverMap,
    AutonWaypointEditor,
    CameraFeed,
    OdometryReading,
    DriveControls,
    MastGimbalControls,
  },

  data() {
    return {
      odom: null as Odom | null,

      basestationOdom: null as Odom | null,

      teleopEnabledCheck: false,

      ledColor: 'bg-danger', //red

      stuck_status: false,

      navState: 'OffState',

      cameraFeedEnabled: true,
    }
  },

  computed: {
    ...mapState('websocket', {
      scienceMessage: (state: WebSocketState) => state.messages['science'],
      navMessage: (state: WebSocketState) => state.messages['nav'],
    }),

    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled',
      teleopEnabled: 'teleopEnabled',
    }),
  },

  watch: {
    scienceMessage(msg) {
      if (msg.type == 'led') {
        if (msg.red)
          this.ledColor = 'bg-danger' //red
        else if (msg.green)
          this.ledColor = 'blink' //blinking green
        else if (msg.blue) this.ledColor = 'bg-primary' //blue
      }
    },
    navMessage(msg) {
      if (msg.type == 'nav_state') {
        this.navState = msg.state
      }
    },
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    toggleFeed() {
      this.cameraFeedEnabled = !this.cameraFeedEnabled
    },
    updateOdom(odom: Odom) {
      this.odom = odom
    },
    updateBasestationOdom(odom: Odom) {
      this.basestationOdom = odom
    },
  },

  topics: ['auton', 'drive', 'nav', 'science', 'waypoints'],

  mounted() {
    for (const topic of this.$options.topics) {
      this.$store.dispatch('websocket/setupWebSocket', topic)
    }
    this.$store.dispatch('websocket/sendMessage', {
      id: 'waypoints',
      message: {
        type: 'get_auton_waypoint_list',
      },
    })
  },

  unmounted() {
    for (const topic of this.$options.topics)
      this.$store.dispatch('websocket/closeWebSocket', topic)
  },

  beforeUnmount() {
    this.ledColor = 'bg-white'
  },
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
