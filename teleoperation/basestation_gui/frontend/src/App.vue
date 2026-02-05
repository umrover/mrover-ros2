<template>
  <div id="app">
    <NavBar />
    <router-view />
  </div>
</template>

<script lang="ts">
import { onMounted } from 'vue'
import NavBar from './views/NavBar.vue'
import { useThemeStore } from '@/stores/theme'

export default {
  name: 'App',
  components: {
    NavBar,
  },
  setup() {
    const themeStore = useThemeStore()
    onMounted(() => {
      themeStore.initTheme()
    })
  }
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
  background-color: var(--view-bg);
  overflow: hidden;
  box-sizing: border-box;
}

.island {
  background-color: var(--island-bg);
  border-radius: 6px;
}

h1, h2, h3, h4, h5, h6 {
  font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', 'Monaco', monospace;
  font-weight: 600;
  color: var(--text-primary);
  letter-spacing: -0.04em;
  margin: 0;
  padding: 0;
  line-height: 1.2;
}

.vue-grid-layout {
  height: 100% !important;
  width: 100% !important;
}

.vue-grid-item {
  touch-action: none;
}

.vue-grid-item > div {
  height: 100%;
  width: 100%;
}
</style>
