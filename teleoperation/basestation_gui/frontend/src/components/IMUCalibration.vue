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

<script lang="ts">
import Vuex from 'vuex'
const { mapState } = Vuex
import LEDIndicator from './LEDIndicator.vue'

const calibration_limit = 3

export default {
  components: {
    LEDIndicator,
  },

  data() {
    return {
      mag_calibration: 0,
      gyro_calibration: 0,
      accel_calibration: 0,
      calibration_limit_master: calibration_limit,
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  watch: {
    message(msg) { // DEPRECATED, not updating to new style
      switch (msg.type) {
        case 'calibration':
          this.mag_calibration = msg.magnetometer_calibration
          this.gyro_calibration = msg.gyroscope_calibration
          this.accel_calibration = msg.acceleration_calibration
          break
      }
    },
  },
}
</script>
<style scoped>
.wrap {
  display: flex;
  justify-content: center;
  height: 100%;
}
</style>
