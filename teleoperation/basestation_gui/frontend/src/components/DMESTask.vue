<template>
  <div :class="type === 'ES' ? 'wrapper-es' : 'wrapper-dm'">
    <div class='shadow p-3 mb-5 header'>
      <h1 v-if="type === 'ES'">ES GUI Dashboard</h1>
      <h1 v-else>DM GUI Dashboard</h1>
      <a class='logo' href="/"><img src='/mrover.png' alt='MRover' title='MRover' width='200' /></a>
    </div>

    <div v-if="type === 'DM'" class='shadow p-3 rounded odom'>
      <OdometryReading :odom='odom' />
    </div>
    <div v-if="type === 'DM'" class='shadow p-3 rounded map'>
      <BasicMap :odom='odom' />
    </div>
    <div v-if="type === 'DM'" class='shadow p-3 rounded waypoint-editor'>
      <BasicWaypointEditor :odom='odom' />
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
      <ControllerDataTable msg-type='drive_left_state' header='Left Drive States' />
      <ControllerDataTable msg-type='drive_right_state' header='Right Drive States' />

      <div v-if="type === 'ES'" class="auton-typing">
        <div class="auton-typing-input">
          <h4>Autonomous Typing Task</h4>
          <AutonTyping :mission="'auton'"/>
        </div>
        <div class="feedback-section">
          <h4>Feedback</h4>
          <table>
            <thead>
              <tr>
                <th>Key</th>
                <th>State</th>
              </tr>
            </thead>
            <tbody>
              <tr v-for="(state, key) in feedback" :key="key">
                <td>{{ key }}</td>
                <td>{{ state }}</td>
              </tr>
            </tbody>
          </table>
        </div>
        <div class="alignment-section">
          <h4>Planar Alignment</h4>
          <p>{{ alignmentDegrees }} degrees</p>
        </div>
      </div>
    </div>
    <div v-show='false'>
      <MastGimbalControls />
    </div>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import { mapActions, mapState } from 'vuex'
import ControllerDataTable from './ControllerDataTable.vue'
import ArmControls from './ArmControls.vue'
import AutonTyping from './AutonTyping.vue'
import BasicMap from './BasicRoverMap.vue'
import BasicWaypointEditor from './BasicWaypointEditor.vue'
import OdometryReading from './OdometryReading.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
import Rover3D from './Rover3D.vue'
import { quaternionToMapAngle } from '../utils'

export default defineComponent({
  components: {
    ControllerDataTable,
    ArmControls,
    AutonTyping,
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
      // Default coordinates at MDRS
      odom: {
        latitude_deg: 38.406025,
        longitude_deg: -110.7923723,
        bearing_deg: 0,
        altitude: 0
      }
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if (msg.type == 'gps_fix') {
        this.odom.latitude_deg = msg.latitude
        this.odom.longitude_deg = msg.longitude
        this.odom.altitude = msg.altitude
      } else if (msg.type == 'orientation') {
        this.odom.bearing_deg = quaternionToMapAngle(msg.orientation)
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    cancelIK: function() {
      this.sendMessage({ type: 'cancel_click_ik' })
    }
  },

  created: function() {
    window.addEventListener('keydown', (event: KeyboardEvent) => {
      if (event.key !== ' ') return

      this.cancelIK(event)
    })
  }
})
</script>

<style>
.auton-typing {
  display: flex;
  flex-direction: row;
  margin-top: 20px;
  justify-content: space-between;
}

.feedback-section table {
  width: 100%;
  border-collapse: collapse;
}

.feedback-section th, .feedback-section td {
  padding: 8px;
  border: 1px solid #ddd;
}

.wrapper-dm {
  display: grid;
  gap: 10px;
  grid-template-columns: 50% 50%;
  grid-template-areas:
    'header header'
    'arm-controls arm-controls'
    'map waypoint-editor'
    'map odom';
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
  left: 50%;
  transform: translateX(-50%);
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
