<template>
  <div id="app">
    <NavBar />
    <router-view />
  </div>
</template>

<script lang="ts">
import NavBar from './views/NavBar.vue'

export default {
  name: 'App',
  components: {
    NavBar,
  },
}
</script>

<style>
html,
body {
  height: 100%;
  margin: 0;
  overflow: hidden;
}

#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  display: flex;
  flex-direction: column;
  height: 100%;
  overflow: hidden;
}

.view-wrapper {
  flex: 1;
  min-height: 0;
  width: 100%;
  padding: 0.625rem;
  background-color: #dddddd;
  overflow: hidden;
  box-sizing: border-box;
}

.island {
  background-color: #ffffff;
  border-radius: 6px;
}

h1, h2, h3, h4, h5, h6 {
  font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', 'Monaco', monospace;
  font-weight: 600;
  color: #1a1a1a;
  letter-spacing: -0.04em;
  margin: 0;
  padding: 0;
  line-height: 1.2;
}

</style>
