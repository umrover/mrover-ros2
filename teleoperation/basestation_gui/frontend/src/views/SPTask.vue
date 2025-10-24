<template>
  <div class="main-wrapper view-wrapper">
    <div class="island p-0 rounded map overflow-hidden">
      <BasicMap :odom="odom" />
    </div>
    <div class="island p-2 rounded odom">
      <OdometryReading @odom="updateOdom" class="rounded border border-2 p-2" />
    </div>
    <div class="island p-2 rounded controller_state d-flex gap-2">
      <ControllerDataTable
        msg-type="drive_state"
        header="Drive States"
        class="rounded border border-2 p-2"
      />
      <ControllerDataTable
        msg-type="sa_state"
        header="SA States"
        class="rounded border border-2 p-2"
      />
    </div>
    <div class="island p-3 rounded waypoints">
      <BasicWaypointEditor :odom="odom" />
    </div>
    <div class="island p-2 rounded controls d-flex gap-2">
      <HexHub
        @selectSite="updateSite"
        @orientation="updateOrientation"
        class="border border-2 rounded"
      />
      <PanoCam class="border border-2 rounded" />
      <DriveControls />
      <MastGimbalControls />
    </div>
    <div class="island p-2 rounded sensors">
      <SensorData :site="siteSelect" />
    </div>
  </div>
</template>

<script lang="ts">
import SensorData from '../components/SensorData.vue'
import BasicMap from '../components/BasicRoverMap.vue'
import BasicWaypointEditor from '../components/BasicWaypointEditor.vue'
import DriveControls from '../components/DriveControls.vue'
import MastGimbalControls from '../components/MastGimbalControls.vue'
import OdometryReading from '../components/OdometryReading.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import HexHub from '../components/HexHub.vue'
import PanoCam from '../components/PanoCam.vue'
import Vuex from 'vuex'
const { mapState, mapActions } = Vuex

interface Odom {
  latitude_deg: number
  longitude_deg: number
  bearing_deg: number
}

export default {
  components: {
    SensorData,
    BasicMap,
    BasicWaypointEditor,
    DriveControls,
    MastGimbalControls,
    OdometryReading,
    ControllerDataTable,
    HexHub,
    PanoCam,
  },

  data() {
    return {
      odom: null as Odom | null,
      siteSelect: 0,
      orientation: true,
      site_to_radians: {
        0: 0.0,
        1: (2 * Math.PI) / 5,
        2: (4 * Math.PI) / 5,
        3: (6 * Math.PI) / 5,
        4: (8 * Math.PI) / 5,
      },
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    updateOdom(odom: Odom) {
      this.odom = odom
    },
    updateSite(selectedSite: number) {
      this.siteSelect = selectedSite
      this.$store.dispatch('websocket/sendMessage', {
        id: 'science',
        message: {
          type: 'set_gear_diff_pos',
          position: this.site_to_radians[this.siteSelect],
          isCCW: this.orientation,
        },
      })
    },
    updateOrientation(orientation: boolean) {
      this.orientation = orientation
      this.$store.dispatch('websocket/sendMessage', {
        id: 'science',
        message: {
          type: 'set_gear_diff_pos',
          position: this.site_to_radians[this.siteSelect],
          isCCW: this.orientation,
        },
      })
    },
  },

  mounted: function () {
    this.$store.dispatch('websocket/setupWebSocket', 'arm')
    this.$store.dispatch('websocket/setupWebSocket', 'mast')
    this.$store.dispatch('websocket/setupWebSocket', 'nav')
    this.$store.dispatch('websocket/setupWebSocket', 'science')
    this.$store.dispatch('websocket/setupWebSocket', 'waypoints')
  },

  unmounted: function () {
    this.$store.dispatch('websocket/closeWebSocket', 'arm')
    this.$store.dispatch('websocket/closeWebSocket', 'mast')
    this.$store.dispatch('websocket/closeWebSocket', 'nav')
    this.$store.dispatch('websocket/closeWebSocket', 'science')
    this.$store.dispatch('websocket/closeWebSocket', 'waypoints')
  },
}
</script>

<style scoped>
.main-wrapper {
  display: grid;
  grid-gap: 10px;
  width: 100%;
  height: 100%;
  /* grid-template-columns: 40%; */
  grid-template-rows: 120px;
  grid-template-areas:
    'controls map map'
    'odom map map'
    'sensors sensors waypoints'
    'controller_state controller_state waypoints';
  font-family: sans-serif;
}

.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

.controls {
  grid-area: controls;
}

.sensors {
  grid-area: sensors;
}

.odom {
  grid-area: odom;
}

.controller_state {
  grid-area: controller_state;
}
</style>
