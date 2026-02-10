<template>
  <div class="wrapper">
    <div class="ps-3 pe-2 py-2 d-flex justify-content-between align-items-center position-relative">
      <a class="logo" href="/"><img src="/mrover.png" alt="MRover" title="MRover" width="200" /></a>
      <div class="d-flex align-items-center gap-3">
        <h1>{{ title }}</h1>
        <div v-if="showGridControls" class="d-flex align-items-center gap-1 border border-2 rounded px-2 py-1">
          <button
            class="btn btn-sm cmd-btn-icon-sm border-2"
            :class="gridLayoutStore.locked ? 'btn-danger' : 'btn-success'"
            @click="gridLayoutStore.toggleLock()"
          >
            <i :class="gridLayoutStore.locked ? 'bi bi-lock-fill' : 'bi bi-unlock-fill'"></i>
          </button>
          <button
            class="btn btn-sm btn-secondary cmd-btn-icon-sm border-2"
            @click="gridLayoutStore.triggerReset()"
          >
            <i class="bi bi-arrow-counterclockwise"></i>
          </button>
          <span class="ms-1 small text-muted">Grid</span>
        </div>
      </div>
      <div class="d-flex align-items-stretch gap-2">
        <WebsocketStatus />
        <div class="border-start border-2 align-self-center nav-divider"></div>
        <div class="dropdown d-flex">
          <button
            class="theme-btn border border-2 rounded"
            data-bs-toggle="dropdown"
            aria-expanded="false"
          >
            <i :class="themeIcon"></i>
          </button>
          <ul class="dropdown-menu dropdown-menu-end">
            <li>
              <button
                class="dropdown-item d-flex align-items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'light' }"
                @click="themeStore.setTheme('light')"
              >
                <i class="bi bi-sun-fill"></i> Light
              </button>
            </li>
            <li>
              <button
                class="dropdown-item d-flex align-items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'dark' }"
                @click="themeStore.setTheme('dark')"
              >
                <i class="bi bi-moon-fill"></i> Dark
              </button>
            </li>
            <li>
              <button
                class="dropdown-item d-flex align-items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'high-contrast-light' }"
                @click="themeStore.setTheme('high-contrast-light')"
              >
                <i class="bi bi-circle-half"></i> High Contrast Light
              </button>
            </li>
            <li>
              <button
                class="dropdown-item d-flex align-items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'high-contrast-dark' }"
                @click="themeStore.setTheme('high-contrast-dark')"
              >
                <i class="bi bi-circle-fill"></i> High Contrast Dark
              </button>
            </li>
            <li>
              <button
                class="dropdown-item d-flex align-items-center gap-2 text-danger"
                :class="{ active: themeStore.currentTheme === 'dont-click-me' }"
                @click="themeStore.setTheme('dont-click-me')"
              >
                <i class="bi bi-exclamation-triangle-fill"></i> Don't Click Me
              </button>
            </li>
          </ul>
        </div>
        <NotificationCenter />
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import WebsocketStatus from '../components/WebsocketStatus.vue';

export default defineComponent({
  name: 'NavBar',
  computed: {
    title(): string {
      // @ts-expect-error ts dumb
      return this.getTitleForRoute(this.$route.path)
    },
  },
  components: {
    WebsocketStatus,
    NotificationCenter
  },
  methods: {
    getTitleForRoute(path: string): string {
      const routeTitles: Record<string, string> = {
        '/': 'Menu',
        '/AutonTask': 'Autonomy Mission',
        '/DMTask': 'Delivery Mission',
        '/ESTask': 'Equipment Servicing',
        '/ISHTask': 'ISH Mission',
        '/SATask': 'Sample Acquisition',
        '/Cameras': 'Camera View',
        '/dev': 'Development View'
      };

      return routeTitles[path] || 'Unknown Path';
    },
  }
});
</script>

<style scoped>
h1 {
	font-family: 'Consolas', 'Courier New', 'DejaVu Sans Mono', monospace;
  font-weight: 400;
  font-size: 2.7rem;
	letter-spacing: -0.1rem;
  margin: 0;
  user-select: none;
}

.logo {
  position: absolute;
  left: 48%;
}

h1 {
  font-size: clamp(1.25rem, 1rem + 0.5vw, 1.75rem) !important;
  margin: 0 !important;
  line-height: 1 !important;
}

.nav-divider {
  height: clamp(32px, 2.5vw, 48px);
}

.theme-btn {
  min-width: 48px;
  height: 100%;
  padding: 0.25rem 0.5rem;
  background-color: var(--card-bg);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  cursor: pointer;
}

.theme-btn i {
  font-size: 1.25rem;
}
</style>
