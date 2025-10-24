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

/* rules below targets all button classes and ensures the text is black
.btn {
  color: #000 !important;
}

.btn-outline-primary:hover,
.btn-outline-primary:focus,
.btn-outline-secondary:hover,
.btn-outline-secondary:focus 
{
  color: #000 !important;
}

.btn-primary.active,
.btn-primary:active,
.btn:is(.btn-primary, .btn-outline-primary):focus 
{
  color: #000 !important;
} */
</style>
