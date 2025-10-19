<template>
  <div class="flex w-100">
    <h2>White LED</h2>
    <ToggleButton
      :current-state="siteEnabled[site]"
      :label-enable-text="'LED Site ' + String.fromCharCode(site + 65)"
      :label-disable-text="'LED Site ' + String.fromCharCode(site + 65)"
      @change="toggleLEDs()"
    />
    <!-- <LEDIndicator
          :connected="siteEnabled[site]"
          :name="'LED Status'"
          :show_name="true"
      /> -->
  </div>
</template>

<script lang="ts">
import Vuex from 'vuex'
const { mapActions } = Vuex
import ToggleButton from './ToggleButton.vue'
// import LEDIndicator from "./LEDIndicator.vue";

export default {
  components: {
    ToggleButton,
    // LEDIndicator
  },

  props: {
    site: {
      type: Number,
      required: true,
    },
  },

  data() {
    return {
      siteEnabled: [false, false],
    }
  },

  // computed: {
  // ...mapState('websocket', ['message'])
  // },

  // watch: {
  //     message(msg) {
  //         if (msg.type == 'white_leds' && !msg.success) {
  //             this.siteEnabled[this.site] = !this.siteEnabled[this.site];
  //             alert('Toggling Auto Shutdown failed.')
  //         }
  //     },
  // },

  methods: {
    async toggleLEDs() {
      this.siteEnabled[this.site] = !this.siteEnabled[this.site]

      try {
        const { scienceAPI } = await import('../utils/api')
        const siteName = this.site === 0 ? 'a' : 'b'
        await scienceAPI.setWhiteLEDs(siteName, this.siteEnabled[this.site])
      } catch (error) {
        console.error('Failed to toggle white LEDs:', error)
        // Revert state on error
        this.siteEnabled[this.site] = !this.siteEnabled[this.site]
      }
    },
  },
}
</script>
