<template>
  <div class="wrapper">
    <div class="shadow p-3 mb-5 header">
      <a class='logo' href="/"><img src='/mrover.png' alt='MRover' title='MRover' width='200' /></a>
      <h1>ISH Dashboard</h1>
      <a href='/'>
        <img class='logo' src='/mrover.png' alt='MRover' title='MRover' width='200' style="cursor: pointer;" />
      </a>
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
    <!-- TODO: remove Cholorophyll (and maybe component) and replace with a new sensors component -->
    <div class="shadow p-3 rounded chlorophyll">
      <Chlorophyll />
    </div>
    <div class="shadow p-3 rounded ninhydrin">
      <NinhydrinBenedict :site="site" :isNinhydrin="true" />
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
    'chlorophyll chlorophyll';
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
</style>
