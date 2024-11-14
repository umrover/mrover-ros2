<template>
  <div class="shadow p-3 mb-5 header">
    <h1>Cameras Dashboard</h1>
    <img class='logo' src='/mrover.png' alt='MRover' title='MRover' width='200' />
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
<div v-for = "cam in cameras[selectedMission]" :key="cam">
  <!-- TODO: figure out ID -->
  <CameraFeed :mission="selectedMission" :id="0" :name="cam"></CameraFeed>
</div>
</template>

<script lang="ts">
import CameraSelection from '../components/CameraSelection.vue'
import CameraFeed from './CameraFeed.vue'
import { mapActions, mapState } from 'vuex'
import { reactive } from 'vue'

export default {
  components: {
    CameraSelection,
    CameraFeed
  },

  data() {
    return {
      percent: 0,
      missionType: ["DM Mission", "ES Mission", "ISH GUI", "Sample Acquisition GUI", "Autonomy Mission"],
      selectedMission: "",
      cameras: {
        "DM Mission": ["Cam1" ,"Cam2"],
        "ES Mission": ["Cam3" ,"Cam4","Cam5"],
        "ISH GUI": ["Cam6" ,"Cam7"],
        "Sample Acquisition GUI": ["Cam8" ,"Cam9"],
        "Autonomy Mission": ["Cam10" ,"Cam11"]
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
</style>