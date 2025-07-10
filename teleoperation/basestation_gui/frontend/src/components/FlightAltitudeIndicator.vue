<template>
  <div>
    <Altitude :size="200" :pitch="pitch" :roll="roll" />
  </div>
</template>

<script lang="ts">
import { Altitude } from 'vue-flight-indicators'
import Vuex from 'vuex'
const { mapState } = Vuex

export default {
  components: {
    Altitude,
  },
  data() {
    return {
      // Degrees
      pitch: 0,
      roll: 0,
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  watch: {
    message(msg) {
      if (msg.type == 'orientation') {
        const [qx, qy, qz, qw] = msg.orientation
        this.pitch = (Math.asin(2 * (qx * qz - qy * qw)) * 180) / Math.PI
        this.roll =
          (Math.atan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)) *
            180) /
          Math.PI
      }
    },
  },
}
</script>

<style scoped></style>
