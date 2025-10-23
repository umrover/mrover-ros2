<template>
  <div class="calibration-wrapper">
    <Checkbox :name="name" @toggle="toggleCalibration" />
    <span class="led">
      <LEDIndicator :connected="calibrated" :name="name" :show_name="false" />
    </span>
  </div>
</template>

<!-- unused component -->

<script lang="ts" setup>
import { defineComponent, ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'
import Checkbox from './BasicCheckbox.vue'
import LEDIndicator from './LEDIndicator.vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

const props = defineProps({
  name: {
    type: String,
    required: true,
  },
  topic_name: {
    type: String,
    required: true,
  },
})

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const toggleEnabled = ref(false)
const calibrated = ref(false)
let interval: number | undefined = undefined

const message = computed(() => messages.value[props.topic_name])

watch(message, (msg) => {
  if (msg && msg.type == 'calibrate_motors') {
    if (toggleEnabled.value) {
      if (Array.isArray(msg.result) && msg.result.length > 0) {
        toggleEnabled.value = false
        for (let j = 0; j < msg.result.length; ++j) {
          alert('ESW cannot calibrate motor ' + msg.result[j])
        }
      } else if (typeof msg.result === 'string') {
        toggleEnabled.value = false
        alert('ESW cannot calibrate motor ' + msg.result)
      } else calibrated.value = true
    }
  }
})

watch(toggleEnabled, (val) => {
  // When the checkbox is toggled, publish a single false request to the calibrate service
  if (!val) {
    publishCalibrationMessage()
  }
})

onBeforeUnmount(() => {
  clearInterval(interval)
  toggleEnabled.value = false
  publishCalibrationMessage()
})

onMounted(() => {
  interval = setInterval(() => {
    if (!calibrated.value && toggleEnabled.value) {
      publishCalibrationMessage()
    }
  }, 200)
})

const toggleCalibration = () => {
  toggleEnabled.value = !toggleEnabled.value
}

const publishCalibrationMessage = () => {
  websocketStore.sendMessage(props.topic_name, { type: 'calibrate_motors' })
}
</script>

<style>
.calibration-wrapper {
  padding: 1% 0 1% 0;
  display: flex;
  flex-direction: row;
}

.led {
  margin-left: 5%;
  display: block;
}
</style>
