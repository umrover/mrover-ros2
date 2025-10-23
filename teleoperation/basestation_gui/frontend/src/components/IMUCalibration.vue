<template>
  <div class="rounded bg-white d-flex flex-row align-items-center gap-2">
    <span class="fw-bold ms-2">IMU Calibration</span>
    <div class="d-flex gap-2 border border-2 rounded p-1">
      <span>Magnetometer</span>
      <LEDIndicator
        :name="mag_calibration.toString()"
        :show_name="true"
        :connected="mag_calibration == calibration_limit_master"
      />
    </div>
    <div class="d-flex gap-2 border border-2 rounded p-1">
      <span>Gyroscope</span>
      <LEDIndicator
        :name="gyro_calibration.toString()"
        :show_name="true"
        :connected="gyro_calibration == calibration_limit_master"
      />
    </div>
    <div class="d-flex gap-2 border border-2 rounded p-1">
      <span>Accelerometer</span>
      <LEDIndicator
        :name="accel_calibration.toString()"
        :show_name="true"
        :connected="accel_calibration == calibration_limit_master"
      />
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import LEDIndicator from './LEDIndicator.vue'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const mag_calibration = ref(0)
const gyro_calibration = ref(0)
const accel_calibration = ref(0)
const calibration_limit_master = 3

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (msg && msg.type === 'calibration') {
    mag_calibration.value = msg.magnetometer_calibration
    gyro_calibration.value = msg.gyroscope_calibration
    accel_calibration.value = msg.acceleration_calibration
  }
})
</script>
<style scoped>
.wrap {
  display: flex;
  justify-content: center;
  height: 100%;
}
</style>
