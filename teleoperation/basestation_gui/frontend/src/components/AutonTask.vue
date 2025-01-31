<template>
  <div class='wrapper'>
    <div class='shadow p-3 mb-5 header'>
      <h1>Auton Dashboard</h1>
      <a href='/'>
        <img class='logo' src='/mrover.png' alt='MRover' title='MRover' width='200' style="cursor: pointer;" />
      </a>
    </div>
    <div :class="['shadow p-3 rounded data', ledColor]">
      <h2>Nav State: {{ navState }}</h2>
      <OdometryReading @odom='updateOdom' />
    </div>
    <div class='shadow p-3 rounded feed'> <!-- meant to be cost mapb -->
      <button @click="toggleFeed" class="btn btn-primary mb-2">
        {{ cameraFeedEnabled ? 'Disable' : 'Enable' }} Camera Feed
      </button>
      <div v-if="cameraFeedEnabled" class='camera-container'>
        <CameraFeed :mission="'ZED'" :id='0' :name="'ZED'"/>
        <p v-if="cameraFeedEnabled">Camera Feed On</p>
      </div>
    </div>
    <div class='shadow p-3 rounded map'>
      <AutonRoverMap :odom='odom' />  
    </div>
    <div class='shadow p-3 rounded waypoints'>
      <AutonWaypointEditor :odom='odom' @toggleTeleop='teleopEnabledCheck = $event' />
    </div>
    <!--Enable the drive controls if auton is off-->
    <div v-if='!autonEnabled && teleopEnabledCheck' v-show='false' class='driveControls'>
      <DriveControls />
      <MastGimbalControls></MastGimbalControls>
    </div>
    <div class='conditions'>
      <div v-if='!stuck_status' class='shadow p-3 rounded bg-success text-center'>
        <h4>Nominal Conditions</h4>
      </div>
      <div v-else class='shadow p-3 rounded bg-danger text-center'>
        <h4>Obstruction Detected</h4>
      </div>
    </div>
    <div class='shadow p-3 rounded moteus'>
      <ControllerDataTable msg-type='drive_left_state' header='Drive Left States' />
      <ControllerDataTable msg-type='drive_right_state' header='Drive Right States' />
    </div>
  </div>
</template>

<script lang='ts'>
import { mapActions, mapGetters, mapState } from 'vuex'
import AutonRoverMap from './AutonRoverMap.vue'
import AutonWaypointEditor from './AutonWaypointEditor.vue'
import CameraFeed from './CameraFeed.vue'
import OdometryReading from './OdometryReading.vue'
import DriveControls from './DriveControls.vue'
import MastGimbalControls from './MastGimbalControls.vue'
import ControllerDataTable from './ControllerDataTable.vue'
import { defineComponent } from 'vue'

let interval: number

interface Odom {
  latitude_deg: number;
  longitude_deg: number;
  bearing_deg: number;
}

export default defineComponent({
  components: {
    ControllerDataTable,
    AutonRoverMap,
    AutonWaypointEditor,
    CameraFeed,
    OdometryReading,
    DriveControls,
    MastGimbalControls
  },

  data() {
    return {
      odom: null as Odom | null,

      teleopEnabledCheck: false,

      ledColor: 'bg-danger', //red

      stuck_status: false,

      navState: 'OffState',

      cameraFeedEnabled: true
    }
  },

  computed: {
    ...mapState('websocket', ['message']),

    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled',
      teleopEnabled: 'teleopEnabled'
    })
  },

  watch: {
    message(msg) {
      if (msg.type == 'led') {
        if (msg.red) this.ledColor = 'bg-danger' //red
        else if (msg.green) this.ledColor = 'blink' //blinking green
        else if (msg.blue) this.ledColor = 'bg-primary' //blue
      } else if (msg.type == 'nav_state') {
        this.navState = msg.state
      }
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    toggleFeed(){
      this.cameraFeedEnabled = !this.cameraFeedEnabled
    },
    updateOdom(odom: Odom) {
      this.odom = odom;
    }
  },

  beforeUnmount: function() {
    this.ledColor = 'bg-white'
    window.clearInterval(interval)
  }
})
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: auto 30% 30%;
  grid-template-areas:
    'header header header'
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

.header {
  grid-area: header;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
}

.logo {
  position: absolute;
  left: 44.45%;
  transform: translateX(-50%);
  transform: translateY(-50%);
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

.btn{
  display: block;
  margin: 10px auto;
}

.camera-container{
  display: flex;
  flex-direction: column;
  align-items: center;
}

</style>
