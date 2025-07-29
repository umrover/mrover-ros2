<template>
  <div class="view-wrapper">
    <!-- TODO -->
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex
import type { WebSocketState } from '../types/websocket'

export default defineComponent({
  components: {
		// TODO
	},

  mounted() {
    this.$store.dispatch('websocket/setupWebSocket', 'waypoints')
		// TODO
  },

  unmounted() {
    this.$store.dispatch('websocket/closeWebSocket', 'waypoints')
		// TODO
  },

  computed: {
    ...mapState('websocket', {
      waypointsMessage: (state: WebSocketState) => state.messages['waypoints'],
    }),
  },

  watch: {
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
