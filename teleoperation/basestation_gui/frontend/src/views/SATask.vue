<template>
  <div class="wrapper view-wrapper overflow-x-hidden h-100">
    <div class="island p-2 rounded controls d-flex gap-2">
      <SAArmControls :currentSite="siteSelect" class="border boder-2 rounded"/>
      <HexHub @selectSite="updateSite" @orientation="updateOrientation" class="border boder-2 rounded"/>
      <LSActuator class="border boder-2 rounded"/>
      <PanoCam class="border boder-2 rounded"/>
      <DriveControls />
      <MastGimbalControls />
    </div>
    <div class="island p-0 rounded map overflow-hidden">
      <BasicMap :odom="odom" />
    </div>
    <div class="island p-3 rounded waypoints">
      <BasicWaypointEditor :odom="odom" />
    </div>
    <div class="island p-1 rounded data d-flex gap-2">
      <OdometryReading @odom="updateOdom" class="rounded border border-2 p-2"/>
      <ControllerDataTable msg-type="drive_state" header="Drive States" class="rounded border border-2 p-2"/>
      <ControllerDataTable msg-type="sa_state" header="SA States" class="rounded border border-2 p-2"/>
      <SoilData class="rounded border border-2 p-2"/>
    </div>
  </div>
</template>

<script lang="ts">
import Vuex from 'vuex'
const { mapState } = Vuex
import BasicMap from '../components/BasicRoverMap.vue'
import SoilData from '../components/SoilData.vue'
import BasicWaypointEditor from '../components/BasicWaypointEditor.vue'
import DriveControls from '../components/DriveControls.vue'
import MastGimbalControls from '../components/MastGimbalControls.vue'
import OdometryReading from '../components/OdometryReading.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import SAArmControls from '../components/SAArmControls.vue'
import HexHub from '../components/HexHub.vue'
import LSActuator from '../components/LSActuator.vue'
import PanoCam from '../components/PanoCam.vue'

interface Odom {
  latitude_deg: number
  longitude_deg: number
  bearing_deg: number
}

export default {
  components: {
    ControllerDataTable,
    BasicMap,
    SoilData,
    BasicWaypointEditor,
    DriveControls,
    MastGimbalControls,
    SAArmControls,
    OdometryReading,
    HexHub,
    LSActuator,
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
    updateOdom(odom: Odom) {
      this.odom = odom
    },
    async updateSite(selectedSite: number) {
      this.siteSelect = selectedSite

      try {
        const { scienceAPI } = await import('../utils/api')
        await scienceAPI.setGearDiffPosition(
          this.site_to_radians[this.siteSelect],
          this.orientation
        )
      } catch (error) {
        console.error('Failed to set gear differential position:', error)
      }
    },
    async updateOrientation(orientation: boolean) {
      this.orientation = orientation

      try {
        const { scienceAPI } = await import('../utils/api')
        await scienceAPI.setGearDiffPosition(
          this.site_to_radians[this.siteSelect],
          this.orientation
        )
      } catch (error) {
        console.error('Failed to set gear differential position:', error)
      }
    },
  },

  mounted: function () {
    this.$store.dispatch('websocket/setupWebSocket', 'arm')
    this.$store.dispatch('websocket/setupWebSocket', 'mast')
    this.$store.dispatch('websocket/setupWebSocket', 'nav')
    this.$store.dispatch('websocket/setupWebSocket', 'science')
  },

  unmounted: function () {
    this.$store.dispatch('websocket/closeWebSocket', 'arm')
    this.$store.dispatch('websocket/closeWebSocket', 'mast')
    this.$store.dispatch('websocket/closeWebSocket', 'nav')
    this.$store.dispatch('websocket/closeWebSocket', 'science')
  },
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 55% auto; 
  grid-template-rows: auto 1fr auto;
  grid-template-areas:
    'controls controls'
    'map waypoints'
    'data data';
  font-family: sans-serif;
  height: auto;
  width: 100%;
}

.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

.data {
  grid-area: data;
}

.soilData {
  grid-area: soilData;
}

.controls {
  grid-area: controls;
}
</style>
