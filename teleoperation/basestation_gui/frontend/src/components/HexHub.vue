<template>
  <div>
    <!-- Creates the buttons for each site (A-F) -->
    <div class="row">
      <div v-for="index in 6" :key="index" class="col-sm">
        <div class="form-check d-flex justify-content-center align-items-center">
          <input
            v-model="currentSite"
            class="btn-check"
            type="radio"
            :id="'Site' + index"
            :value="siteList[index]"
          />
          <label class="btn btn-outline-secondary" :for="'Site' + index">Site {{ siteList[index] }}</label>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { mapState, mapActions } from 'vuex';

export default {
  data() {
    return {
      missionType: [
        "DM Mission", 
        "ES Mission", 
        "ISH GUI", 
        "Sample Acquisition GUI", 
        "Autonomy Mission"
      ],
      selectedMission: "DM Mission",
      currentSite: 1, // Default site is 1 (Site A)
      siteList: { 1: "Site 1", 2: "Site 2", 3: "Empty", 4: "Cache", 5: "Closed 1", 6: "Closed 2" },
      camsEnabled: [],
      camsStreaming: []
    };
  },
  computed: {
    ...mapState('websocket', ['message']),
  },
  methods: {
    ...mapActions('websocket', ['sendMessage']),
    
    sendHexHubSite() {
      // Send the selected site (A-F) to the backend
      this.sendMessage({ type: 'hexHubSite', site: this.currentSite });
    },
  },
};
</script>

<style scoped>
/* Add your styles here */
</style>