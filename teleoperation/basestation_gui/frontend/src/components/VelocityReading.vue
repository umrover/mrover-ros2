<template>
  <div class="velocity-wrapper">
    <h4>Velocity</h4>
    <div class="velocity-row pb-1">
      <span class="label">Linear:</span>
      <span class="value">{{ linear_x.toFixed(3) }} m/s</span>
    </div>
    <div class="velocity-row">
      <span class="label">Angular:</span>
      <span class="value">{{ angular_z.toFixed(3) }} rad/s</span>
    </div>
  </div>
</template>

<script lang="ts">
import Vuex from 'vuex';
const { mapState } = Vuex;
export default {
  data() {
    return {
      linear_x: 0,
      angular_z: 0
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      if(msg.type == 'cmd_vel') {
        this.linear_x = msg.linear.x;
        this.angular_z = msg.angular.z;
      }
    }
  }
}
</script>

<style scoped>
.velocity-wrapper {
  background-color: #dddddd;
  padding: 0.5rem;
  border-radius: 8px;
  max-width: 320px;
}

h3 {
  font-family: monospace;
  letter-spacing: -0.1rem;
}

.velocity-row {
  display: flex;
  flex-direction: column;
}

.label {
  font-weight: bold;
  letter-spacing: -0.03rem;
}
</style>
