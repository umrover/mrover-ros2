<template>
  <div class="p-2">
    <h3 class="m-0 p-0">Limit Switch</h3>
    <div class="m-0 p-0">
      <ToggleButton
        id="ls"
        :current-state="lsstate"
        :label-enable-text="'Limit Switch Sensor Actuator'"
        :label-disable-text="'Limit Switch Sensor Actuator'"
        @change="togglels()"
      />
    </div>
  </div>
</template>

<script lang="ts">
import ToggleButton from './ToggleButton.vue'

export default {
  components: {
    ToggleButton,
  },
  data() {
    return {
      lsstate: [false, false],
    }
  },

  methods: {
    async togglels() {
      this.lsstate = !this.lsstate

      try {
        const { scienceAPI } = await import('../utils/api')
        await scienceAPI.setLimitSwitch(this.lsstate)
      } catch (error) {
        console.error('Failed to toggle limit switch:', error)
        // Revert state on error
        this.lsstate = !this.lsstate
      }
    },
  },

  created() {
    this.togglels()
  },
}
</script>