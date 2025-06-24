<template>
  <div id="app">
    <NavBar/>
    <router-view/>
  </div>
</template>

<script lang="ts">
import NavBar from './views/NavBar.vue'

export default {
  name: 'App',
  components: {
    NavBar
  },
}
</script>

<style>
html, body {
  height: 100%;
  margin: 0;
}

#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  display: flex;
  flex-direction: column;
  height: 100%;
}

.dashboard-title {
  text-decoration: none;
  color: black;
}
</style>
