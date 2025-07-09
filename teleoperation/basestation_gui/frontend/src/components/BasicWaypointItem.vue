<template>
  <div
    class="my-1 p-3 rounded waypoint-item"
    v-bind:class="{ 'drone-waypoint': waypoint.drone }"
  >
    <div class="name">
      <p>{{ waypoint.name }}</p>
    </div>
    <div class="location">
      <p>{{ waypoint.lat }}ºN, {{ waypoint.lon }}ºW</p>
    </div>
    <div class="text-center">
      <button class="btn btn-danger mx-2" @click="$emit('delete', { index: index })">Delete</button>
      <button class="btn btn-success mx-2" @click="$emit('find', { index: index })">Find</button>
      <button class="btn btn-success mx-2" @click="$emit('search', {index: index})">Search</button>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapGetters } = Vuex

export default defineComponent({
  props: {
    waypoint: {
      type: Object,
      required: true
    },
    index: {
      type: Number,
      required: true
    }
  },

  computed: {
    ...mapGetters('erd', {
      highlightedWaypoint: 'highlightedWaypoint'
    })
  }
})
</script>

<style scoped>
.drone-waypoint {
  background-color: limegreen;
}
</style>