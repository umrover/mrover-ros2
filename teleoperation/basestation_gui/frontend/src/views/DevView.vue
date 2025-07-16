<template>
  <div class="view-wrapper">
    <ArmControls class="island p-2"/>
    <!-- <Rover3D /> -->
  </div>
</template>

<script lang="ts">
import Rover3D from '../components/Rover3D.vue';
import ArmControls from '../components/ArmControls.vue';
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex
import type { WebSocketState } from '../types/websocket';

export default defineComponent({
  components: {
    Rover3D,
    ArmControls
  },

  mounted() {
    this.$store.dispatch('websocket/setupWebSocket', 'arm')
    this.$store.dispatch('websocket/setupWebSocket', 'nav')

    setTimeout(() => {
      this.$store.dispatch('websocket/closeWebSocket', 'nav')
      this.spamTestMessages()
    }, 1000)
  },

  unmounted() {
    this.$store.dispatch('websocket/closeWebSocket', 'arm')
    this.$store.dispatch('websocket/closeWebSocket', 'nav')
  },
  
  computed: { // correct websocket message receiver, specify websocket in []
    ...mapState('websocket', {
      waypointsMessage: (state: WebSocketState) => state.messages['waypoints']
    }),
  },

  watch: { // then watch for messages
    waypointsMessage(msg) {
      console.log(msg)
    }
  },

  methods: {
    spamTestMessages() {
      const interval = setInterval(() => {
        this.$store.dispatch('websocket/sendMessage', {
          id: 'waypoints',
          message: {
            type: 'debug',
            timestamp: new Date().toISOString(),
          },
        })
      }, 1000)
      setTimeout(() => clearInterval(interval), 5000)
    },
  },
})
</script>
