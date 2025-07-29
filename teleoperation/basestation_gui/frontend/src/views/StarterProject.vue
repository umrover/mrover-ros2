<template>
  <div class="view-wrapper">
    <div class="d-flex flex-col gap-2 mb-2">
      <button class="btn btn-primary" @click="spamTestMessages">
        send websocket messages
      </button>
      <ArmControls class="island py-2" />
    </div>
    <Rover3D class="island m-0 p-0" style="max-height: 700px;" />
  </div>
</template>

<script lang="ts">
import ArmControls from '../components/ArmControls.vue'
import Rover3D from '../components/Rover3D.vue'
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex
import type { WebSocketState } from '../types/websocket'

export default defineComponent({
  components: {
    ArmControls,
    Rover3D,
  },

  mounted() {
    this.$store.dispatch('websocket/setupWebSocket', 'arm')
    this.$store.dispatch('websocket/setupWebSocket', 'waypoints')
  },

  unmounted() {
    this.$store.dispatch('websocket/closeWebSocket', 'arm')
    this.$store.dispatch('websocket/closeWebSocket', 'waypoints')
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
