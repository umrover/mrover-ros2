<template>
  <div :class="type === 'ES' ? 'wrapper-es' : 'wrapper-dm'">
    <div class='shadow p-3 mb-5 header' style="display: flex; align-items: center; justify-content: space-between;">
      <h1 v-if="type === 'ES'">ES GUI Dashboard</h1>
      <h1 v-else>DM GUI Dashboard</h1>
      <a class='logo' href="/"><img src='/mrover.png' alt='MRover' title='MRover' width='200' /></a>
    </div>

    <div v-if="type === 'DM'" class='shadow p-3 rounded odom'>
      <OdometryReading @odom='updateOdom' @drone_odom="updateDroneOdom" />
    </div>
    <div v-if="type === 'DM'" class='shadow p-3 rounded map'>
      <BasicMap :odom='odom' :drone_odom="drone_odom" />
    </div>
    <div v-if="type === 'DM'" class='shadow p-3 rounded waypoint-editor'>
      <BasicWaypointEditor :odom='odom' :droneWaypointButton='true'/>
    </div>
    <div>
      <DriveControls />
    </div>
    <div class='shadow p-3 rounded arm-controls'>
      <ArmControls />
    </div>
    <div class='shadow p-3 rounded rover-3d'>
      <Rover3D />
    </div>
    <div class='shadow p-3 rounded controller_state'>
      <ControllerDataTable msg-type='arm_state' header='Arm States' />
      <ControllerDataTable msg-type='drive_state' header='Drive States' />
    </div>
    <div v-show='false'>
      <MastGimbalControls />
    </div>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import ControllerDataTable from './ControllerDataTable.vue'
import ArmControls from './ArmControls.vue'
import BasicMap from './BasicRoverMap.vue'
import BasicWaypointEditor from './BasicWaypointEditor.vue'
import OdometryReading from './OdometryReading.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
import Rover3D from './Rover3D.vue'

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
      odom:  null as Odom | null,
      drone_odom:  null as DroneOdom | null
    }
  },

  methods: {
    updateOdom(odom: Odom) {
      this.odom = odom;
    },
    updateDroneOdom(odom: DroneOdom) {
      this.drone_odom = odom;
    }
  }

  // methods: {
  //   ...mapActions('websocket', ['sendMessage']),
  //   cancelIK: function() {
  //     this.sendMessage({ type: 'cancel_click_ik' })
  //   }
  // },

  // created: function() {
  //   window.addEventListener('keydown', (event: KeyboardEvent) => {
  //     if (event.key !== ' ') return

  //     this.cancelIK(event)
  //   })
  // }
})
</script>

<style>
.wrapper-dm {
  display: grid;
  gap: 10px;
  grid-template-columns: 50% 50%;
  grid-template-areas:
    'header header'
    'arm-controls arm-controls'
    'map waypoint-editor'
    'map odom'
    'controller_state rover-3d';
  font-family: sans-serif;
  height: auto;
}

.wrapper-es {
  display: grid;
  gap: 10px;
  grid-template-columns: repeat(2, auto);
  grid-template-areas:
    'header header'
    'arm-controls arm-controls'
    'controller_state rover-3d';
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

.logo {
  position: absolute;
  left: 45%;
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
