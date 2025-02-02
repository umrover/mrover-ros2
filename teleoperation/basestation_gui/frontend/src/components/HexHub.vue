<template>
    <h3>HexHub Controls</h3>
      <div class="row">
        <div v-for="name, index in siteList" :key="index" class="col-4">
          <div class="form-check d-flex justify-content-center align-items-center">
            <input
              v-model="currentSite"
              class="btn-check"
              type="radio"
              :id="'Site' + index"
              :value="index"
              @change="sendHexHubSite"
            />
            <label class="btn btn-outline-secondary" :for="'Site' + index">{{ name }}</label>
          </div>
        </div>
      </div>
  </template>
  <script lang="ts">
  import { mapState, mapActions } from 'vuex';
  export default {
    data() {
      return {
        currentSite: 1, // Default site is 1 (Site A)
        siteList: ["A", "B", "C", "D", "E", "F"],
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
  .btn {
    margin-top: 15px;
    width: 150px;
    height: 45px;
    text-align: center !important;
  }

  .btn:hover {
    transform: translateY(-4px);
    box-shadow: 0 6px 10px rgba(0, 0, 0, 0.2);
  }
  </style>
    
