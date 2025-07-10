<template>
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

  // watch: {
  //     message(msg) {
  //         if (msg.type == 'white_leds' && !msg.success) {
  //             this.siteEnabled[this.site] = !this.siteEnabled[this.site];
  //             alert('Toggling Auto Shutdown failed.')
  //         }
  //     },
  // },

  // computed: {
  // ...mapState('websocket', ['message'])
  // },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    toggleLEDs: function () {
      this.siteEnabled[this.site] = !this.siteEnabled[this.site]
			this.$store.dispatch('websocket/sendMessage', {
				id: 'science',
				message: {
					type: 'white_leds',
        site: this.site,
        enabled: this.siteEnabled[this.site],
				},
			})
    },
  },
}
</script>

<style scoped></style>
