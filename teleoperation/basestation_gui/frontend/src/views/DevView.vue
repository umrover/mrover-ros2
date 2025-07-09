<template>
  <OdometryReading />
</template>

<script lang="ts">
import OdometryReading from '../components/OdometryReading.vue'
import { defineComponent } from 'vue'

export default defineComponent({
  components: {
    OdometryReading,
  },

  mounted() {
    this.$store.dispatch('websocket/setupWebSocket', 'general')
    this.$store.dispatch('websocket/setupWebSocket', 'sa')

    setTimeout(() => {
      this.spamTestMessages()
    }, 1000)
  },

  unmounted() {
    this.$store.dispatch('websocket/closeWebSocket', 'general')
    this.$store.dispatch('websocket/closeWebSocket', 'sa')
  },

  methods: {
    spamTestMessages() {
      const interval = setInterval(() => {
        this.$store.dispatch('websocket/sendMessage', {
          id: 'general',
          message: {
            type: 'test',
            timestamp: new Date().toISOString(),
          },
        })
      }, 1000)

      // Optional: stop after 10 seconds
      setTimeout(() => clearInterval(interval), 10000)
    },
  },
})
</script>
