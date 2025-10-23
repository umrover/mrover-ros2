<template>
  <!-- creates the buttons for each mission -->
  <div class="row">
    <div v-for="(mission, index) in missionType" :key="index" class="col-sm">
      <div class="form-check d-flex justify-content-center align-items-center">
        <input
          v-model="selectedMission"
          class="btn-check"
          type="radio"
          :id="'mission' + index"
          :value="mission"
        />
        <label class="btn btn-outline-secondary" :for="'mission' + index">{{ mission }}</label>
      </div>
    </div>
  </div>

  <!-- for each mission, buttons are created for each camera associated with the selected mission -->
  <div class="row sticky-top bg-white">
      <div
        v-for="(camera, index) in cameras[selectedMission]"
        :key="camera"
        class="col d-flex justify-content-center my-5"
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
      <!-- if there is only one camera streaming, take up the whole width, otherwise have two cameras on each row -->
      <div
        :class="['col-12', (camsStreaming.length > 1) ? 'col-md-6' : '', 'camera-feed-container']"
        v-for="index in camsStreaming"
        :key="index"
      >
        <CameraFeed
          :mission="selectedMission"
          :id="index"
          :name="cameras[selectedMission][index]"
        ></CameraFeed>
      </div>
    </div>
  </div> 
</template>

<script lang="ts" setup>
import CameraFeed from '../components/CameraFeed.vue'
import ToggleButton from "../components/ToggleButton.vue";
import { ref, watch, onMounted } from 'vue';

const percent = ref(0)
const missionType = ref(["DM Mission", "ES Mission", "SPI GUI", "Sample Acquisition GUI", "Autonomy Mission"])
const selectedMission = ref("DM Mission")
const cameras = ref({
  "DM Mission": ["Cam1","Cam2"],
  "ES Mission": ["Cam3" ,"Cam4","Cam5"],
  "ISH GUI": ["Cam2" ,"Cam3", "Cam6", "Cam7",],
  "Sample Acquisition GUI": ["Cam8"],
  "Autonomy Mission": ["Cam9" ,"Cam1"]
})
const camsEnabled = ref<boolean[]>([])
const camsStreaming = ref<number[]>([])

watch(selectedMission, (newMission: string) => {
  camsEnabled.value = new Array(cameras.value[newMission].length).fill(true);
  camsStreaming.value = [...Array(camsEnabled.value.length).keys()];
});

onMounted(() => {
  camsEnabled.value = new Array(cameras.value[selectedMission.value].length).fill(true);
  camsStreaming.value = [...Array(camsEnabled.value.length).keys()];
});

const toggleCamera = (idx: number) => {
  camsEnabled.value[idx] = !camsEnabled.value[idx];

  if(camsEnabled.value[idx]) { //add camera feed
    camsStreaming.value.push(idx);
  }
  else { // remove camera feed
    camsStreaming.value = camsStreaming.value.filter((x: number) => x != idx);
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