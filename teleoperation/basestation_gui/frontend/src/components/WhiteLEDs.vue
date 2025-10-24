<template>
  <div class="flex w-100">
    <h2>White LED</h2>
    <ToggleButton
      :current-state="siteEnabled[0]"
      :label-enable-text="'LED Site A'"
      :label-disable-text="'LED Site A'"
      @change="toggleLEDs(0)"
    />

    <ToggleButton
      :current-state="siteEnabled[1]"
      :label-enable-text="'LED Site B'"
      :label-disable-text="'LED Site B'"
      @change="toggleLEDs(1)"
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

  /*props: {
    site: {
      type: Number,
      required: true,
    },
  },*/

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
    ...mapActions('websocket', ['sendMessage']),

    toggleLEDs: function (site_num: number) {
      this.siteEnabled[site_num] = !this.siteEnabled[site_num]
      this.$store.dispatch('websocket/sendMessage', {
        id: 'science',
        message: {
          type: 'white_leds',
          site: site_num,
          enable: this.siteEnabled[site_num],
        },
      })
    },
  },
}
</script>
