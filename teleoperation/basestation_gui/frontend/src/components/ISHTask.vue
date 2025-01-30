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

    <!-- TODO: create a sensor vue file (for the table) -->
    <div class="shadow p-3 rounded sensors">
      <div class="sensors-container">
        <table class="sensors-table table-bordered">
          <thead>
            <tr class="table-primary">
              <!-- empty for alignment -->
              <th></th>
              <th>Sensor 1</th>
              <th>Sensor 2</th>
              <th>Sensor 3</th>
              <th>Sensor 4</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <th scope="row">Site 1</th>
              <td>voltage</td>
              <td>voltage</td>
              <td>voltage</td>
              <td>voltage</td>
            </tr>
            <tr>
              <th scope="row">Site 2</th>
              <td>voltage</td>
              <td>voltage</td>
              <td>voltage</td>
              <td>voltage</td>
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
//   import MCUReset from "./MCUReset.vue"

export default {
  components: {
    SelectSite,
    Chlorophyll,
    NinhydrinBenedict,
    NetworkMonitor,
    //   MCUReset,
  },

  data() {
    return {
      site: 0 as number,
      primary: false
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
  grid-template-columns: repeat(2, auto);
  grid-template-areas:
    'header header'
    'siteSelect siteSelect'
    'ninhydrin benedicts'
    'sensors sensors';
  font-family: sans-serif;
  height: auto;
}

.comms {
  margin-right: 5px;
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
