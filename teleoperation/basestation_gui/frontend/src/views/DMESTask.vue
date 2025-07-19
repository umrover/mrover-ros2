<template>
  <div class="view-wrapper" :class="type === 'ES' ? 'wrapper-es' : 'wrapper-dm'">
    <div v-if="type === 'DM'" class='island p-2 rounded odom'>
      <OdometryReading @odom='updateOdom' @drone_odom="updateDroneOdom" />
    </div>
    <div v-if="type === 'DM'" class='island p-0 rounded map overflow-hidden'>
      <BasicMap :odom='odom' :drone_odom="drone_odom" />
    </div>
    <div v-if="type === 'DM'" class='island p-2 rounded waypoint-editor'>
      <BasicWaypointEditor :odom='odom' :droneWaypointButton='true'/>
    </div>
    <div class='island p-2 rounded arm-controls'>
      <ArmControls />
    </div>
    <div class='island rounded rover-3d overflow-hidden'>
      <Rover3D class="w-100 h-100"/>
    </div>
    <div class='island p-2 rounded controller_state d-flex flex-column gap-2'>
      <ControllerDataTable msg-type='arm_state' header='Arm States' />
      <ControllerDataTable msg-type='drive_state' header='Drive States' />
    </div>
    <div>
      <MastGimbalControls />
      <DriveControls />
    </div>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import ControllerDataTable from '../components/ControllerDataTable.vue'
import ArmControls from '../components/ArmControls.vue'
import BasicMap from '../components/BasicRoverMap.vue'
import BasicWaypointEditor from '../components/BasicWaypointEditor.vue'
import OdometryReading from '../components/OdometryReading.vue'
import DriveControls from '../components/DriveControls.vue'
import MastGimbalControls from '../components/MastGimbalControls.vue'
import Rover3D from '../components/Rover3D.vue'

interface Odom {
  latitude_deg: number;
  longitude_deg: number;
  bearing_deg: number;
}

interface DroneOdom {
  latitude_deg: number;
  longitude_deg: number;
}

export default defineComponent({
  components: {
    ControllerDataTable,
    ArmControls,
    BasicMap,
    BasicWaypointEditor,
    OdometryReading,
    DriveControls,
    MastGimbalControls,
    Rover3D
  },

  props: {
    type: {
      type: String,
      required: true
    }
  },

  data() {
    return {
      odom: null as Odom | null,
      drone_odom: null as DroneOdom | null
    }
  },

  methods: {
    updateOdom(odom: Odom) {
      this.odom = odom;
    },
    updateDroneOdom(odom: DroneOdom) {
      this.drone_odom = odom;
    }
  },

  mounted: function() {
    this.$store.dispatch('websocket/setupWebSocket', 'arm')
    this.$store.dispatch('websocket/setupWebSocket', 'drive')
    this.$store.dispatch('websocket/setupWebSocket', 'mast')
    this.$store.dispatch('websocket/setupWebSocket', 'nav')
    this.$store.dispatch('websocket/setupWebSocket', 'waypoints')
  },

  unmounted: function() {
    this.$store.dispatch('websocket/closeWebSocket', 'arm')
    this.$store.dispatch('websocket/closeWebSocket', 'drive')
    this.$store.dispatch('websocket/closeWebSocket', 'waypoints')
    this.$store.dispatch('websocket/closeWebSocket', 'nav')
    this.$store.dispatch('websocket/closeWebSocket', 'mast')
  },
})
</script>

<style>

.wrapper-dm {
  display: grid;
  gap: 10px;
  width: 100%;
  height: 100%;
  grid-template-columns: 45% 250px auto;
  grid-template-rows: 45% 1fr 1fr 1fr;
  grid-template-areas:
    'rover-3d map map'
    'rover-3d controller_state waypoint-editor'
    'arm-controls controller_state waypoint-editor'
    'odom controller_state waypoint-editor';
  font-family: sans-serif;
}

.wrapper-es {
  display: grid;
  gap: 10px;
  width: 100%;
  height: 100%;
  grid-template-columns: 400px auto;
  grid-template-rows: 50% auto;
  grid-template-areas:
    'arm-controls rover-3d'
    'controller_state rover-3d';
  font-family: sans-serif;
}

.map {
  grid-area: map;
}

.odom {
  grid-area: odom;
}

.waypoint-editor {
  grid-area: waypoint-editor;
}

.arm-controls {
  grid-area: arm-controls;
}

.controller_state {
  grid-area: controller_state;
}

.rover-3d {
  grid-area: rover-3d;
}
</style>
