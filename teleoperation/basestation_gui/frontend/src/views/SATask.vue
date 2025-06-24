<template>
  <div class='wrapper view-wrapper'>
    <div class='island p-3 rounded map'>
      <BasicMap :odom='odom' />
    </div>
    <div class='island p-3 rounded waypoints'>
      <BasicWaypointEditor :odom='odom' />
    </div>
    <div class='island p-3 rounded soilData'>
      <SoilData />
    </div>
    <div>
      <DriveControls />
    </div>
    <div class='island p-3 rounded arm'>
      <SAArmControls :currentSite="siteSelect" /> 
    </div>
    <div class='island p-3 rounded moteus'>
      <ControllerDataTable msg-type='drive_state' header='Drive States' />
      <ControllerDataTable msg-type='sa_state' header='SA States' />
    </div>
    <div v-show='false'>
      <MastGimbalControls />
    </div>
    <div class='island p-3 rounded odom'>
      <OdometryReading @odom='updateOdom'/>
    </div>
    <div class="island p-3 rounded hexHub">
      <HexHub @selectSite="updateSite" @orientation="updateOrientation"/>
    </div>
    <div class="island p-3 rounded lsActuator">
      <LSActuator />
    </div>
  </div>
</template>

<script lang='ts'>
import { mapState, mapActions } from 'vuex';
import BasicMap from '../components/BasicRoverMap.vue'
import SoilData from '../components/SoilData.vue'
import BasicWaypointEditor from '../components/BasicWaypointEditor.vue'
import DriveControls from '../components/DriveControls.vue'
import MastGimbalControls from '../components/MastGimbalControls.vue'
import OdometryReading from '../components/OdometryReading.vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import SAArmControls from '../components/SAArmControls.vue'
import NetworkMonitor from "../components/NetworkMonitor.vue"
import HexHub from '../components/HexHub.vue'
import LSActuator from '../components/LSActuator.vue'

interface Odom {
  latitude_deg: number;
  longitude_deg: number;
  bearing_deg: number;
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
    NetworkMonitor,
    OdometryReading,
    HexHub,
    LSActuator
  },
  data() {
    return {
      odom:  null as Odom | null,
      siteSelect: 0,
      orientation: true,
      site_to_radians: 
      {
        0: 0.0,
        1: (2*Math.PI)/5,
        2: (4*Math.PI)/5,
        3: (6*Math.PI)/5,
        4: (8*Math.PI)/5
      }
    }
  },
  computed: {
      ...mapState('websocket', ['message']),
    },
  methods: {
    ...mapActions('websocket', ['sendMessage']),
    updateOdom(odom: Odom) {
      this.odom = odom;
    },
    updateSite(selectedSite: number) {
      this.siteSelect = selectedSite; 
      this.sendMessage(
        {
          type: "set_gear_diff_pos",
          position: this.site_to_radians[this.siteSelect],
          isCCW: this.orientation
        }
      )
    },
    updateOrientation(orientation: boolean) {
      this.orientation = orientation
      this.sendMessage(
        {
          type: "set_gear_diff_pos",
          position: this.site_to_radians[this.siteSelect],
          isCCW: this.orientation
        }
      )
    }
  }
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 50% repeat(2, auto);
  grid-template-areas:
    'arm lsActuator soilData'
    'map hexHub waypoints'
    'map odom odom'
    'moteus moteus moteus';
  font-family: sans-serif;
  height: auto;
  width: 100%;
}

.dashboard-title {
  color: black;
  text-decoration: none;
}

.dashboard-title:hover {
  color: darkgray;
}

.network {
  float: right;
}

.map {
  grid-area: map;
}

.waypoints {
  grid-area: waypoints;
}

.arm {
  grid-area: arm;
}

.moteus {
  grid-area: moteus;
}

.odom {
  grid-area: odom;
}

.soilData {
  grid-area: soilData;
}

.hexHub {
  grid-area: hexHub;
}

.lsActuator {
  grid-area: lsActuator;
}
</style>
