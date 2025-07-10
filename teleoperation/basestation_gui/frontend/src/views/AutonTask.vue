<template>
  <div class="wrapper view-wrapper">
    <div :class="['island p-3 rounded data', ledColor]">
      <h2>Nav State: {{ navState }}</h2>
      <OdometryReading
        @odom="updateOdom"
        @basestation_odom="updateBasestationOdom"
      />
    </div>
    <div class="island p-3 rounded feed">
      <!-- meant to be cost map -->
      <button @click="toggleFeed" class="btn btn-primary mb-2">
        {{ cameraFeedEnabled ? 'Disable' : 'Enable' }} Camera Feed
      </button>
      <div v-if="cameraFeedEnabled" class="camera-container">
        <CameraFeed :mission="'ZED'" :id="0" :name="'ZED'" />
        <p v-if="cameraFeedEnabled">Camera Feed On</p>
      </div>
    </div>
    <div class="island p-0 rounded map">
      <AutonRoverMap :odom="odom" :basestation="basestationOdom" />
    </div>
    <div class="island p-3 rounded waypoints">
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
    <div class="conditions">
      <div
        v-if="!stuck_status"
        class="island p-3 rounded bg-success text-center"
      >
        <h4>Nominal Conditions</h4>
      </div>
      <div v-else class="island p-3 rounded bg-danger text-center">
        <h4>Obstruction Detected</h4>
      </div>
    </div>
    <div class="island p-3 rounded moteus">
      <!-- drive_left_state and drive_right_state not found -->
      <ControllerDataTable
        msg-type="drive_left_state"
        header="Drive Left States"
      />
      <ControllerDataTable
        msg-type="drive_right_state"
        header="Drive Right States"
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
      navMessage: (state: WebSocketState) => state.messages['nav']
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
    }
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

  mounted: async function () {
    window.setTimeout(() => {
      for (const topic of this.$options.topics)
        this.$store.dispatch('websocket/setupWebSocket', topic)
    }, 500)
  },

  unmounted: function () {
    for (const topic of this.$options.topics)
      this.$store.dispatch('websocket/closeWebSocket', topic)
  },

  beforeUnmount: function () {
    this.ledColor = 'bg-white'
  },
})
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: auto 30% 30%;
  grid-template-areas:
    'feed map waypoints'
    'data data waypoints'
    'data data conditions'
    'moteus moteus moteus';

  font-family: sans-serif;
  height: auto;
  width: auto;
}

.blink {
  animation: blinkAnimation 1s infinite;
  /* Blinks green every second */
}

@keyframes blinkAnimation {
  0%,
  100% {
    background-color: var(--bs-success);
  }

  50% {
    background-color: var(--bs-white);
  }
}
h2 {
  padding: 2px;
  margin: 0px;
}

.comms {
  margin-right: 5px;
}

/* Grid area declarations */
.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

.conditions {
  grid-area: conditions;
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

.btn {
  display: block;
  margin: 10px auto;
}

.camera-container {
  display: flex;
  flex-direction: column;
  align-items: center;
}
</style>
