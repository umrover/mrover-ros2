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
  /* stylelint-disable-next-line declaration-property-value-disallowed-list */
  overflow: hidden;
}

#app {
  /* stylelint-disable declaration-property-value-disallowed-list */
  display: flex;
  flex-direction: column;
  height: 100%;
  overflow: hidden;
  /* stylelint-enable declaration-property-value-disallowed-list */
  font-family: Avenir, Helvetica, Arial, sans-serif;
}

.view-wrapper {
  box-sizing: border-box;
  flex: 1;
  width: 100%;
  min-height: 0;
  /* stylelint-disable-next-line declaration-property-value-disallowed-list */
  overflow: hidden;
  background-color: var(--view-bg);
}

.island {
  background-color: var(--island-bg);
  border-radius: var(--cmd-radius-md);
}

h1, h2, h3, h4, h5, h6 {
  padding: 0;
  margin: 0;
  font-family: 'JetBrains Mono', 'Fira Code', Consolas, Monaco, monospace;
  font-weight: 600;
  line-height: 1.2;
  color: var(--text-primary);
  letter-spacing: -0.04em;
}

.vue-grid-layout {
  width: 100% !important;
  height: 100% !important;
}

.vue-grid-item {
  touch-action: none;
}

.vue-grid-item > div {
  width: 100%;
  height: 100%;
}
</style>
