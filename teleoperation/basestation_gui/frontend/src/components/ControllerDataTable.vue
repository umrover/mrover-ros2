<template>
  <div class='wrap border border-2 p-2 rounded'>
    <h3 class="m-0 p-0 mb-1">{{ header }}</h3>
    <table class='table table-bordered m-0 p-0 border' style='table-layout: fixed; width: auto'>
      <tbody>
      <tr>
        <th class='table-secondary'>Motor</th>
        <td v-for='(name, i) in name' :key='i' class="m-0 px-1">
          {{ name }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>State</th>
        <td v-for='(state, i) in state' :key='i'>
          {{ state }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>Error</th>
        <td v-for='(error, i) in error' :key='i'>
          {{ error }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>Limit Hit</th>
        <td v-for='(limits, i) in limits' :key='i'>
          {{ limits }}
        </td>
      </tr>
      </tbody>
    </table>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import Vuex from 'vuex';
const { mapState } = Vuex;
import type { WebSocketState } from '../types/websocket';

export default defineComponent({
  props: {
    header: {
      type: String,
      required: true,
    },
    msgType: {
      type: String,
      required: true,
    }
  },

  data() {
    return {
      name: [] as string[],
      state: [] as string[],
      error: [] as string[],
      limits: [] as boolean[]
    }
  },

  computed: {
    ...mapState('websocket', {
      armMessage: (state: WebSocketState) => state.messages['arm'],
      driveMessage: (state: WebSocketState) => state.messages['drive']
    })
  },

  // arm_state, drive_state, sa_state, drive_left_state, drive_right_state
  // arm, drive, 

  watch: {
    armMessage(msg) {
      if (msg.type == this.msgType) {
        this.name = msg.name
        this.state = msg.state
        this.error = msg.error
        this.limits = msg.limit_hit
      }
    },
    driveMessage(msg) {
      if (msg.type == this.msgType) {
        this.name = msg.name
        this.state = msg.state
        this.error = msg.error
        this.limits = msg.limit_hit
      }
    },
  }
})
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
  min-width: 350px;
}
</style>
