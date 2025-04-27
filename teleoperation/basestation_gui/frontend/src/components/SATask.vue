<template>
  <div class='wrapper'>
    <div class='shadow p-3 mb-5 header'>
     <h1><a class='dashboard-title' href="/">SA Dashboard</a></h1>
      <a class='logo' href="/"><img src='/mrover.png' alt='MRover' title='MRover' width='200' /></a>
      <div class="network">
      <NetworkMonitor />
      </div>
    </div>
    <div class='shadow p-3 rounded map'>
      <BasicMap :odom='odom' />
    </div>
    <div class='shadow p-3 rounded waypoints'>
      <BasicWaypointEditor :odom='odom' />
    </div>
    <div class='shadow p-3 rounded soilData'>
      <SoilData />
    </div>
    <div>
      <DriveControls />
    </div>
    <div class='shadow p-3 rounded arm'>
      <SAArmControls :currentSite="siteSelect" /> 
    </div>
    <div class='shadow p-3 rounded moteus'>
      <ControllerDataTable msg-type='drive_state' header='Drive States' />
      <ControllerDataTable msg-type='sa_state' header='SA States' />
    </div>
    <div v-show='false'>
      <MastGimbalControls />
    </div>
    <div class='shadow p-3 rounded odom'>
      <OdometryReading @odom='updateOdom'/>
    </div>
    <div class="shadow p-3 rounded hexHub">
      <HexHub @selectSite="updateSite" @orientation="updateOrientation"/>
    </div>
    <div class="shadow p-3 rounded lsActuator">
      <LSActuator />
    </div>
  </div>
</template>

<script lang='ts'>
import BasicMap from './BasicRoverMap.vue'
import SoilData from './SoilData.vue'
import BasicWaypointEditor from './BasicWaypointEditor.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
import OdometryReading from './OdometryReading.vue'
import ControllerDataTable from './ControllerDataTable.vue'
import SAArmControls from './SAArmControls.vue'
import NetworkMonitor from "./NetworkMonitor.vue"
import HexHub from './HexHub.vue'
import LSActuator from './LSActuator.vue'
import { mapState, mapActions } from 'vuex';

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
        1: Math.PI/2,
        2: Math.PI,
        3: (3*Math.PI)/2,
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
      // !!! MAY NEED TO CHANGE HOW CURRENT SITE IS CONVERTED TO RADIANS
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
    'header header header'
    'arm lsActuator soilData'
    'map hexHub waypoints'
    'map odom odom'
    'moteus moteus moteus';
  font-family: sans-serif;
  height: auto;
  width: 97%;
}

.dashboard-title {
  color: black;
  text-decoration: none;
}

.dashboard-title:hover {
  color: darkgray;
}

.header {
  grid-area: header;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
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
