<template>
  <OdometryReading />
  <WebsocketStatus />
</template>

<script lang="ts">
import OdometryReading from '../components/OdometryReading.vue'
import WebsocketStatus from '../components/WebsocketStatus.vue';
import { defineComponent } from 'vue'

export default defineComponent({
  components: {
    OdometryReading,
    WebsocketStatus,
  },

  mounted() {
    this.$store.dispatch('websocket/setupWebSocket', 'waypoints')

    setTimeout(() => {
      this.spamTestMessages()
    }, 1000)
  },

  unmounted() {
    this.$store.dispatch('websocket/closeWebSocket', 'waypoints')
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

      // Optional: stop after 10 seconds
      setTimeout(() => clearInterval(interval), 5000)
    },
  },
})
</script>
