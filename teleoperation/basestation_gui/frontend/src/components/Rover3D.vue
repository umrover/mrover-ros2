<template>
  <canvas class="webgl p-0 h-100 w-100"></canvas>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex
// @ts-expect-error shut up ts
import threeSetup from '../rover_three.js'

export default defineComponent({
  data() {
    return {
      threeScene: null,
      temp_positions: ['base', 'a', 'b', 'c', 'd', 'e'],
      positions: [],
    }
  },

  mounted() {
    this.threeScene = threeSetup('threejs')
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  watch: {
    message(msg) {
      if (msg.type == 'fk') {
        msg.position = msg.position.map(x => (isNaN(x) ? 0 : x))
        this.threeScene.fk(msg.position)
      } else if (msg.type == 'ik') {
        this.threeScene.ik(msg.target)
      }
    },
  },
})
</script>
