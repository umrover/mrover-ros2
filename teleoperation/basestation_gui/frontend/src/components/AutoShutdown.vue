<template>
  <div class="flex w-100">
    <h2>Heater Auto Shutdown</h2>
    <ToggleButton
      id="autoshutdown"
      :current-state="autoShutdownEnabled"
      :label-enable-text="'Auto Shutdown'"
      :label-disable-text="'Auto Shutdown'"
      @change="sendAutoShutdownCmd()"
    />
    <!-- <LEDIndicator
          :connected="autoShutdownEnabled"
          :name="'Auto Shutdown Status'"
          :show_name="true"
      /> -->
  </div>
</template>

<script lang="ts">
import ToggleButton from './ToggleButton.vue'
// import LEDIndicator from "./LEDIndicator.vue";

export default {
  components: {
    ToggleButton,
    // LEDIndicator
  },

  data() {
    return {
      autoShutdownEnabled: [false, false],
    }
  },

  methods: {
    async sendAutoShutdownCmd() {
      this.autoShutdownEnabled = !this.autoShutdownEnabled

      try {
        const { scienceAPI } = await import('../utils/api')
        await scienceAPI.setAutoShutoff(this.autoShutdownEnabled)
      } catch (error) {
        console.error('Failed to toggle auto shutdown:', error)
        // Revert state on error
        this.autoShutdownEnabled = !this.autoShutdownEnabled
      }
    },
  },

  created() {
    this.sendAutoShutdownCmd()
  },
}
</script>