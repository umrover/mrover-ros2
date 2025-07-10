<template>
  <div class="wrapper view-wrapper">
    <div class="island p-3 rounded row siteSelect">
      <div class="col-4">
        <SelectSite @site="onSiteChange" />
      </div>
      <div class="col-4">
        <WhiteLEDs :site="site" />
      </div>
      <div class="col-4">
        <AutoShutdown />
      </div>
    </div>
    <div class="island p-3 rounded benedicts">
      <NinhydrinBenedict :site="site" :isNinhydrin="false" />
    </div>
    <div class="island p-3 rounded ninhydrin">
      <NinhydrinBenedict :site="site" :isNinhydrin="true" />
    </div>
    <div class="island p-3 rounded container-fluid camera">
      <div class="d-flex justify-content-center">
        <ToggleButton
          :current-state="true"
          label-enable-text="Camera A On"
          label-disable-text="Camera A Off"
          @change="cameraA = $event"
        />
        <ToggleButton
          :current-state="true"
          label-enable-text="Camera B On"
          label-disable-text="Camera B Off"
          @change="cameraB = $event"
        />
      </div>
      <div class="row gx-3 gy-3 justify-content-center">
        <div v-if="cameraA" class="col-12">
          <CameraFeed :mission="'ish'" :id="10" :name="'Sample A'" />
        </div>
        <div v-if="cameraB" class="col-12">
          <CameraFeed :mission="'ish'" :id="11" :name="'Sample B'" />
        </div>
      </div>
    </div>

    <div class="island p-3 rounded sensors">
      <SensorData :site="site" />
    </div>
  </div>
</template>

<script lang="ts">
import SelectSite from '../components/SelectSite.vue'
import NinhydrinBenedict from '../components/NinhydrinBenedict.vue'
import CameraFeed from '../components/CameraFeed.vue'
import ToggleButton from '../components/ToggleButton.vue'
import AutoShutdown from '../components/AutoShutdown.vue'
import SensorData from '../components/SensorData.vue'
import WhiteLEDs from '../components/WhiteLEDs.vue'
import Vuex from 'vuex'
const { mapState } = Vuex

export default {
  components: {
    SelectSite,
    NinhydrinBenedict,
    CameraFeed,
    ToggleButton,
    AutoShutdown,
    SensorData,
    WhiteLEDs,
  },

  data() {
    return {
      site: 0 as number,
      cameraA: true,
      cameraB: true,
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  methods: {
    onSiteChange(value: string) {
      this.site = parseInt(value)
    },
  },

  topics: ['science'],

  mounted() {
    window.setTimeout(() => {
      for (const topic of this.$options.topics)
        this.$store.dispatch('websocket/setupWebSocket', topic)
    }, 0)
  },

  unmounte() {
    this.$store.dispatch('websocket/closeWebSocket', 'science')
  },
}
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  grid-template-columns: repeat(2, auto) 40%;
  grid-template-areas:
    'siteSelect siteSelect camera'
    'ninhydrin benedicts camera'
    'sensors sensors camera';
  font-family: sans-serif;
  height: auto;
}

.dashboard-title {
  color: black;
  text-decoration: none;
}

.dashboard-title:hover {
  color: darkgray;
}

.comms {
  margin-right: 5px;
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

.ninhydrin {
  grid-area: ninhydrin;
}

.camera {
  grid-area: camera;
}

.sensors {
  grid-area: sensors;
}
</style>
