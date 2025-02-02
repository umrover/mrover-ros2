<template>
    <div>
      <div class="row">
        <div v-for="index in 6" :key="index" class="col-4">
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
        siteList: { 1: "A", 2: "B", 3: "C", 4: "D", 5: "E", 6: "F" },
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
  <style>
  .btn {
    margin-top: 15px;
    width: 150px;
    height: 90px;
    text-align: center !important;
  }

  .btn:hover {
    transform: translateY(-4px);
    box-shadow: 0 6px 10px rgba(0, 0, 0, 0.2);
  }
  </style>