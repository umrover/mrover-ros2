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
          :labelEnableText="camera"
          :labelDisableText="camera"
          :currentState="camsEnabled[index]"
          @change="toggleCamera(index)"
        />
      </div>
    </div>

  <div class="container-fluid">
    <div class="row gx-3 gy-3 justify-content-center">
      <div
        :class="['col-12', (numEnabled > 1) ? 'col-md-6' : '']"
        v-for="(cam, index) in cameras[selectedMission]"
        :key="cam"
      >
        <div 
          class="camera-feed-container "
          v-if="camsEnabled[index]"
        >
          <CameraFeed
            :mission="selectedMission"
            :id="index"
            :name="cam"
          ></CameraFeed>
        </div>
      </div>
    </div>
  </div> 
</template>

<script lang="ts">
import CameraFeed from './CameraFeed.vue'
import ToggleButton from "../components/ToggleButton.vue";
import { mapActions, mapState } from 'vuex'
import { reactive } from 'vue'

export default {
  components: {
    ToggleButton,
    CameraFeed
  },

  data() {
    return {
      percent: 0,
      missionType: ["DM Mission", "ES Mission", "ISH GUI", "Sample Acquisition GUI", "Autonomy Mission"],
      selectedMission: "DM Mission",
      cameras: {
        "DM Mission": ["Cam1","Cam2"],
        "ES Mission": ["Cam3" ,"Cam4","Cam5"],
        "ISH GUI": ["Cam2" ,"Cam3", "Cam6", "Cam7",],
        "Sample Acquisition GUI": ["Cam8"],
        "Autonomy Mission": ["Cam9" ,"Cam1"]
      },
      camsEnabled: [], // stores cam enabled state for cameras in selected mission (default true)
      numEnabled: 0
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'pano_feedback') {
        this.percent = msg.percent;
      }
    },

    selectedMission(newMission: string) {
      this.camsEnabled = new Array(this.cameras[newMission].length).fill(true);
    },

    camsEnabled: {
      handler: function() {
      this.numEnabled = this.camsEnabled.filter((x: boolean) => x).length;
      },
      deep: true
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  created: function() {
    this.camsEnabled = new Array(this.cameras[this.selectedMission].length).fill(true);
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    takePanorama() {
      this.sendMessage({ type: 'takePanorama' })
    },

    toggleCamera(idx: number) {
      this.camsEnabled[idx] = !this.camsEnabled[idx];
    }

  }
}
</script>

<style scoped>
.percent {
  font-size: large;
}

.camera-feed-container {
  display: flex;
  justify-content: center;
  align-items: center;
}
</style>