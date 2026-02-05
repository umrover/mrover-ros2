<template>
  <div class="cmd-panel controller-table-panel">
    <div class="cmd-panel-header">
      <h4>{{ header }}</h4>
      <div class="d-flex gap-1">
        <button
          type="button"
          class="btn btn-sm border-2"
          :class="showStatus ? 'btn-success' : 'btn-outline-secondary'"
          @mousedown.prevent
          @click="showStatus = !showStatus"
        >Status</button>
        <button
          type="button"
          class="btn btn-sm border-2"
          :class="showValues ? 'btn-success' : 'btn-outline-secondary'"
          @mousedown.prevent
          @click="showValues = !showValues"
        >Values</button>
      </div>
    </div>
    <div class="overflow-x-auto cmd-scroll">
      <table class="cmd-table compact-table">
        <tbody>
        <tr>
          <th class="sticky-col">Motor</th>
          <td v-for="(n, i) in names" :key="i">{{ n }}</td>
        </tr>
        <tr v-if="showStatus">
          <th class="sticky-col">State</th>
          <td v-for="(s, i) in states" :key="i">{{ s }}</td>
        </tr>
        <tr v-if="showStatus">
          <th class="sticky-col">Error</th>
          <td v-for="(e, i) in errors" :key="i">{{ e }}</td>
        </tr>
        <tr v-if="showStatus">
          <th class="sticky-col">Limit Hit</th>
          <td v-for="(l, i) in limitHits" :key="i">{{ l }}</td>
        </tr>
        <tr v-if="showValues">
          <th class="sticky-col">Position</th>
          <td v-for="(p, i) in positions" :key="i">{{ p.toFixed(2) }}</td>
        </tr>
        <tr v-if="showValues">
          <th class="sticky-col">Velocity</th>
          <td v-for="(v, i) in velocities" :key="i">{{ v.toFixed(2) }}</td>
        </tr>
        <tr v-if="showValues">
          <th class="sticky-col">Current</th>
          <td v-for="(c, i) in currents" :key="i">{{ c.toFixed(2) }}</td>
        </tr>
        </tbody>
      </table>
    </div>
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
.controller-table-panel {
  flex: 1 0 auto;
  min-width: 0;
}

.sticky-col {
  position: sticky;
  left: 0;
  z-index: 1;
  background-color: var(--table-header-bg);
}

.compact-table {
  table-layout: auto;
}

.compact-table th,
.compact-table td {
  white-space: nowrap;
  width: 1%;
  text-align: center;
}
</style>
