<template>

  <div class="shadow p-3 mb-5 header">
    <h1>Cameras Dashboard</h1>
    <a class='logo' href = "/"><img src='/mrover.png' alt='MRover' title='MRover' width='200' /></a>
  </div>

  <div class="row">
    <div v-for="(mission, index) in missionType" :key="index" class="col-sm feed">
      <div class="form-check d-flex justify-content-center align-items-center">
        <input
          v-model="selectedMission"
          class="form-check-input"
          type="radio"
          :id="'mission' + index"
          :value="mission"
        />
        <label class="form-check-label" :for="'mission' + index">{{ mission }}</label>
      </div>
    </div>
  </div>

  <div class="row">
      <div
        v-for="(camera, index) in cameras[selectedMission]"
        :key="camera"
        class="col-sm feed"
      >
        <ToggleButton
          :id="camera"
          :labelEnableText="'Enable Camera'"
          :labelDisableText="'Disable Camera'"
          :currentState="cameraSwitch[camera]"
          @change="toggleCamera"
        />
      </div>
    </div>

  <div class="container-fluid">
    <div class="row gx-3 gy-3 justify-content-center">
      <div v-if="cameras[selectedMission].length == 1">
        <div class="camera-feed-container col-12">
          <CameraFeed
            :mission="selectedMission"
            :id="cameras[selectedMission][0]"
            :name="cameras[selectedMission][0]"
            :class="cameras[selectedMission][0]"
            
          ></CameraFeed>
        </div>

      </div>
      <div
        v-else
        class="col-12 col-md-6"
        v-for="cam in cameras[selectedMission]"
        :key="cam"
      >
        <div class="camera-feed-container ">
          <CameraFeed
            :mission="selectedMission"
            :id="cam"
            :name="cam"
            :class="cam"
            
          ></CameraFeed>
        </div>
      </div>
    </div>
  </div> 
</template>

<script lang="ts">
import CameraSelection from '../components/CameraSelection.vue'
import CameraFeed from './CameraFeed.vue'
import ToggleButton from "../components/ToggleButton.vue";
import { mapActions, mapState } from 'vuex'
import { reactive } from 'vue'

export default {
  components: {
    CameraSelection,
    ToggleButton,
    CameraFeed
  },

  data() {
    return {
      percent: 0,
      missionType: ["DM Mission", "ES Mission", "ISH GUI", "Sample Acquisition GUI", "Autonomy Mission"],
      selectedMission: "DM Mission",
      cameras: {
        "DM Mission": ["Cam1" ,"Cam2"],
        "ES Mission": ["Cam3" ,"Cam4","Cam5"],
        "ISH GUI": ["Cam2" ,"Cam3", "Cam6", "Cam7",],
        "Sample Acquisition GUI": ["Cam8"],
        "Autonomy Mission": ["Cam9" ,"Cam1"]
      },
      cameraSwitch: { // determines whether camera is on or off <.>
        "Cam1": true,
        "Cam2": true,
        "Cam3": true,
        "Cam4": true,
        "Cam5": true,
        "Cam6": true,
        "Cam7": true,
        "Cam8": true,
        "Cam9": true,

      }
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'pano_feedback') {
        this.percent = msg.percent;
      }
    },
    capacity: function (newCap, oldCap) {
      if (newCap < oldCap) {
        const numStreaming = this.streamOrder.filter(index => index != -1)
        const index = numStreaming.length - 1
        this.setCamIndex(numStreaming[index])
      }
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
    color: function() {
      return this.camsEnabled ? 'btn-success' : 'btn-secondary'
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    setCamIndex: function(index: number) {
      // every time a button is pressed, it changes cam status and adds/removes from stream
      this.camsEnabled[index] = !this.camsEnabled[index]
      this.changeStream(index)
    },

    addCameraName: function() {
      this.names[this.cameraIdx] = this.cameraName
    },

    changeStream(index: number) {
      const found = this.streamOrder.includes(index)
      if (found) {
        this.streamOrder.splice(this.streamOrder.indexOf(index), 1)
        this.streamOrder.push(-1)
      } else this.streamOrder[this.streamOrder.indexOf(-1)] = index
    },

    takePanorama() {
      this.sendMessage({ type: 'takePanorama' })
    },

    toggleCamera({ id, state }) {
      this.cameraSwitch[id] = state;
      console.log(`Camera ${id} toggled to ${state ? "ON" : "OFF"}`);
    }
  }
}
</script>

<style scoped>
.cameraselection {
  margin: 10px;
}

.custom-btn {
  width: 150px;
  height: 50px;
}

.info {
  height: 200px;
  overflow-y: auto;
}

.percent {
  font-size: large;
}

.camera-feed-container {
  background-color: #fff;
  display: flex;
  justify-content: center;
  align-items: center;
}
</style>