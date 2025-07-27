<template>
  <div class="view-wrapper">
		<h1> body </h1>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex
import type { WebSocketState } from '../types/websocket'

export default defineComponent({
  components: {
  },

  mounted() {
    this.$store.dispatch('websocket/setupWebSocket', 'waypoints')
  },

  unmounted() {
    this.$store.dispatch('websocket/closeWebSocket', 'waypoints')
  },

  computed: {
    // correct websocket message receiver, specify websocket in []
    ...mapState('websocket', {
      waypointsMessage: (state: WebSocketState) => state.messages['waypoints'],
    }),
  },

  watch: {
    // then watch for messages
    waypointsMessage(msg) {
      console.log(msg)
    },
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
