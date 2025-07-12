<template>
  <div class="d-flex gap-2 align-items-center px-1 py-1 rounded fw-bold fs-5 text-black border border-2">
    <p class="mb-0 ps-2">TX: {{ tx.toFixed(2) }}</p>
    <p class="mb-0 pe-2">RX: {{ rx.toFixed(2) }}</p>
  </div>
</template>

<script lang="ts">
import Vuex from 'vuex'
const { mapState } = Vuex

export default {
  data() {
    return {
      tx: 0,
      rx: 0,
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  watch: {
    message(msg) { // NOT YET IMPLEMENTED / MISSING IMPL
      if (msg.type == 'network_monitor') {
        this.tx = msg.tx
        this.rx = msg.rx
      }
    },
  },
}
</script>