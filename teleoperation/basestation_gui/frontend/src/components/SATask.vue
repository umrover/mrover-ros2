<template>
  <div class='wrapper'>
    <div class='shadow p-3 mb-5 header'>
      <h1>SA Dashboard</h1>
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
      <SAArmControls />
    </div>
    <div class='shadow p-3 rounded moteus'>
      <ControllerDataTable msg-type='drive_state' header='Drive States' />
    </div>
    <div class='shadow p-3 rounded joints'>
      <JointStateDataTable msg-type='sa_joint' header='SA Joints' />
      <JointStateDataTable msg-type='plunger' header='Plunger (w/o offset)' />
    </div>
    <div v-show='false'>
      <MastGimbalControls />
    </div>
    <div class='shadow p-3 rounded odom'>
      <OdometryReading @odom='updateOdom' />
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
import JointStateDataTable from './JointStateDataTable.vue'
import NetworkMonitor from "./NetworkMonitor.vue";

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
    JointStateDataTable
  },
  data() {
    return {
      odom: null as Odom | null
    }
  },

  methods: {
    updateOdom(odom: Odom) {
      this.odom = odom;
    }
  }

}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 50% 50%;
  grid-template-areas:
    'header header'
    'arm soilData'
    'map waypoints'
    'map odom'
    'moteus joints';
  font-family: sans-serif;
  height: auto;
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

.motorData {
  grid-area: motorData;
}

.moteus {
  grid-area: moteus;
}

.joints {
  grid-area: joints;
}

.limit {
  grid-area: limit;
}

.odom {
  grid-area: odom;
}

.soilData {
  grid-area: soilData;
}

.calibration {
  grid-area: calibration;
  display: flex;
  flex-direction: column;
}

.calibration-checkboxes {
  margin: -4% 0 1% 0;
}

ul#vitals li {
  display: inline;
  float: left;
  padding: 0 10px 0 0;
}
</style>
