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
      <ControllerDataTable msg-type='sa_state' header='SA States' />
    </div>
    <div v-show='false'>
      <MastGimbalControls />
    </div>
    <div class='shadow p-3 rounded odom'>
      <OdometryReading :odom='odom'></OdometryReading>
    </div>
    <div class="shadow p-3 rounded hexHub">
      <h3>HexHub Options</h3>
      <div class="d-flex justify-content-center">
        <div v-for="h in hexHubOptions" :key="h" class='form-check mx-3'>
          <input
            v-model='hexHubPos'
            class='form-check-input'
            type='radio'
            :id="'hex'+h"
            :value='h'
          />
          <label class='form-check-label' :for="'hex'+h">{{h}}</label>
        </div>
      </div>
    </div>
    <div class="shadow p-3 rounded pumps">
      <h3>Pump Controls</h3>

    </div>
    <!-- TODO: add pumps -->
    <!-- TODO: add limit switch -->
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
  },
  data() {
    return {
      // Default coordinates at MDRS
      odom: {
        latitude_deg: 38.406025,
        longitude_deg: -110.7923723,
        bearing_deg: 0,
        altitude: 0
      },
      hexHubPos: 0,
      // hexHubOptions: [1,2,3,4]
      hexHubOptions: ["Sample 1","Sample 2","Sample Cache","Empty Soil Deposit"]
    }
  },
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: 50% repeat(2, auto);
  grid-template-areas:
    'header header header'
    'arm hexHub soilData'
    'map pumps waypoints'
    'map odom odom'
    'moteus moteus moteus';
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

.pumps {
  grid-area: pumps;
}
</style>
