<template>
  <div class="wrapper view-wrapper">
    <div class="island p-2 rounded controls d-flex gap-2">
      <SelectSite @site="onSiteChange" class="border border-2 rounded p-2"/>
      <AutoShutdown class="border border-2 rounded p-2"/>
      <WhiteLEDs :site="site" class="border border-2 rounded p-2"/>
    </div>
    <div class="island p-2 rounded benedicts">
      <NinhydrinBenedict :site="site" :isNinhydrin="false" />
    </div>
    <div class="island p-2 rounded ninhydrin">
      <NinhydrinBenedict :site="site" :isNinhydrin="true" />
    </div>
    <div class="island p-2 rounded container-fluid camera">
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

    <div class="island p-2 rounded sensors">
      <SensorData :site="site" />
    </div>
  </div>
</template>

<script lang="ts" setup>
import SelectSite from '../components/SelectSite.vue'
import NinhydrinBenedict from '../components/NinhydrinBenedict.vue'
import CameraFeed from '../components/CameraFeed.vue'
import ToggleButton from '../components/ToggleButton.vue'
import AutoShutdown from '../components/AutoShutdown.vue'
import SensorData from '../components/SensorData.vue'
import WhiteLEDs from '../components/WhiteLEDs.vue'
import { ref, onMounted, onUnmounted } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

const site = ref(0 as number)
const cameraA = ref(true)
const cameraB = ref(true)

const onSiteChange = (value: string) => {
  site.value = parseInt(value)
}

const topics = ['science']
const websocketStore = useWebsocketStore()

onMounted(() => {
  window.setTimeout(() => {
    for (const topic of topics)
      websocketStore.setupWebSocket(topic)
  }, 0)
})

onUnmounted(() => {
  websocketStore.closeWebSocket('science')
})
</script>

<style scoped>
.wrapper {
  display: grid;
  grid-gap: 10px;
  width: 100%;
  height: 100%;
  grid-template-columns: repeat(2, auto) 40%;
  grid-template-areas:
    'controls controls camera'
    'ninhydrin benedicts camera'
    'sensors sensors camera';
  font-family: sans-serif;
}

.benedicts {
  grid-area: benedicts;
}

.controls {
  grid-area: controls;
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
