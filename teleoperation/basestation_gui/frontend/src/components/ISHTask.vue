<template>
  <div class="wrapper">
    <div class="shadow p-3 mb-5 header">
      <a class='logo' href="/"><img src='/mrover.png' alt='MRover' title='MRover' width='200' /></a>
      <h1>ISH Dashboard</h1>
      <div class="network">
        <NetworkMonitor/>
      </div>
    </div>
    <div class="shadow p-3 rounded siteSelect">
      <SelectSite @site="onSiteChange" />
    </div>
    <div class="shadow p-3 rounded benedicts">
      <NinhydrinBenedict :site="site" :isNinhydrin="false" />
    </div>
    <div class="shadow p-3 rounded ninhydrin">
      <NinhydrinBenedict :site="site" :isNinhydrin="true" />
    </div>
    <div class="shadow p-3 rounded container-fluid camera">
      <div class="row gx-3 gy-3 justify-content-center">
        <ToggleButton :current-state="true" label-enable-text="Camera A On"
      label-disable-text="Camera A off" @change="cameraA = $event" />
      <ToggleButton :current-state="true" label-enable-text="Camera B On"
      label-disable-text="Camera B off" @change="cameraB = $event" />
      </div>
      <div class="row gx-3 gy-3 justify-content-center">
        <div v-if="cameraA" class="col-12">
          <CameraFeed
            :mission="'ish'"
            :id="10"
            :name="'Sample A'"
          ></CameraFeed>
        </div>
        <div v-if="cameraB" class="col-12">
          <CameraFeed
            :mission="'ish'"
            :id="11"
            :name="'Sample B'"
          ></CameraFeed>
        </div>
      </div>
    </div>

    <!-- TODO: create a sensor vue file (for the table) -->
    <div class="shadow p-3 rounded sensors">
      <div class="sensors-container">
        <table class="sensors-table table-bordered">
          <thead>
            <tr class="table-primary">
              <!-- empty for alignment -->
              <th></th>
              <th>Oxygen</th>
              <th>Methane</th>
              <th>UV</th>
              <th>Humidity</th>
              <th>Temp °C</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <th scope="row">Site A</th>
              <td>-</td>
              <td>-</td>
              <td>-</td>
              <td>-</td>
              <td>-</td>
            </tr>
            <tr>
              <th scope="row">Site B</th>
              <td>-</td>
              <td>-</td>
              <td>-</td>
              <td>-</td>
              <td>-</td>
            </tr>
          </tbody>  
        </table>
  
        <div class="buttons">
          <p class="example-button"><strong>Generate report</strong></p>
          <p class="example-button"><strong>Generate report</strong></p>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import SelectSite from './SelectSite.vue'
import Chlorophyll from './Chlorophyll.vue'
import NinhydrinBenedict from './NinhydrinBenedict.vue'
import NetworkMonitor from "./NetworkMonitor.vue";
import CameraFeed from './CameraFeed.vue';
import ToggleButton from './ToggleButton.vue';

export default {
  components: {
    SelectSite,
    Chlorophyll,
    NinhydrinBenedict,
    NetworkMonitor,
    CameraFeed,
    ToggleButton
  },

  data() {
    return {
      site: 0 as number,
      cameraA: true,
      cameraB: true,
    }
  },

  methods: {
    onSiteChange(value: string) {
      this.site = parseInt(value)
    }
  }
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: repeat(3, auto);
  grid-template-areas:
    'header header header'
    'siteSelect siteSelect camera'
    'ninhydrin benedicts camera'
    'sensors sensors camera';
  font-family: sans-serif;
  height: auto;
}

.comms {
  margin-right: 5px;
}

.helpscreen {
  z-index: 1000000000;
  display: block;
  visibility: hidden;
  background-color: black;
  opacity: 0.8;
  position: absolute;
  left: 0px;
  top: 0px;
  width: 100%;
  height: 100%;
}

.helpimages {
  z-index: 1000000001;
  visibility: hidden;
  position: absolute;
  left: 5%;
  top: 5%;
  width: 90%;
  height: 90%;
}

.help {
  z-index: 1000000002;
  display: flex;
  float: right;
  opacity: 0.8;
  cursor: auto;
}

.help:hover {
  opacity: 1;
  cursor: pointer;
}

.help:hover ~ .helpscreen,
.help:hover ~ .helpimages {
  visibility: visible;
}

.header {
  grid-area: header;
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
}

.network {
  float: right;
}

.benedicts {
  grid-area: benedicts;
}

.siteSelect {
  grid-area: siteSelect;
}

.chlorophyll {
  grid-area: chlorophyll;
}

.ninhydrin {
  grid-area: ninhydrin;
}

.camera {
  grid-area: camera;
}

.sensors {
  grid-area: sensors;
}

.sensors-container {
  display: flex; /* Aligns the table and buttons in a row */
  align-items: start; /* Aligns the buttons to the top of the table */
  gap: 20px; /* Adds space between the table and buttons */
}

.sensors-table {
  width: 80%; /* Adjust table width as needed */
}

.buttons {
  margin-top: 20px;
}

.example-button {
  color: white;
  ;
  background-color: darkcyan;
  
}
</style>
