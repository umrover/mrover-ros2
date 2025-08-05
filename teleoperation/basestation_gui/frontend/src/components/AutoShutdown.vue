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
import Vuex from 'vuex'
const { mapActions } = Vuex
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

  // watch: {
  //     message(msg) {
  //         if (msg.type == 'auto_shutoff' && !msg.success) {
  //             this.autoShutdownEnabled = !this.autoShutdownEnabled;
  //             alert('Toggling Auto Shutdown failed.')
  //         }
  //     },
  // },

  // computed: {
  // ...mapState('websocket', ['message'])
  // },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    sendAutoShutdownCmd: function () {
      this.autoShutdownEnabled = !this.autoShutdownEnabled
      this.$store.dispatch('websocket/sendMessage', {
        id: 'science',
        message: {
          type: 'auto_shutoff',
          shutoff: this.autoShutdownEnabled,
        },
      })
    },
  },

  created() {
    this.sendAutoShutdownCmd()
  },
}
</script>