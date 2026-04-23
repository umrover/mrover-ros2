<template>
  <div class="wrapper bg-theme-card border-b border-theme">
    <div class="pl-4 pr-2 py-2 flex justify-between items-center relative">
      <a class="logo absolute" href="/"><img src="/mrover.png" alt="MRover" title="MRover" width="200" /></a>
      <div class="flex items-center gap-4">
        <h1 class="text-theme-primary">{{ title }}</h1>
        <div v-if="showGridControls" class="flex items-center gap-1 border border-2 border-theme rounded px-2 py-1">
          <button
            class="cmd-btn cmd-btn-sm cmd-btn-icon-sm"
            :class="gridLayoutStore.locked ? 'cmd-btn-danger' : 'cmd-btn-success'"
            data-testid="pw-grid-lock-btn"
            @click="gridLayoutStore.toggleLock()"
          >
            <i :class="gridLayoutStore.locked ? 'bi bi-lock-fill' : 'bi bi-unlock-fill'"></i>
          </button>
          <button
            class="cmd-btn cmd-btn-sm cmd-btn-secondary cmd-btn-icon-sm"
            data-testid="pw-grid-reset-btn"
            @click="gridLayoutStore.triggerReset()"
          >
            <i class="bi bi-arrow-counterclockwise"></i>
          </button>
          <span class="ml-1 text-sm text-muted">Grid</span>
        </div>
      </div>
      <div class="flex items-stretch gap-2">
        <WebsocketStatus />
        <div class="border-l border-2 border-start-theme self-center nav-divider"></div>
        <div class="dropdown flex relative">
          <button
            class="theme-btn flex flex-col items-center justify-center border border-2 border-theme rounded"
            data-testid="pw-theme-dropdown"
            @click="dropdownOpen = !dropdownOpen"
          >
            <i :class="themeIcon"></i>
          </button>
          <ul
            class="cmd-dropdown-menu"
            :class="{ show: dropdownOpen }"
          >
            <li>
              <button
                class="cmd-dropdown-item flex items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'light' }"
                @click="themeStore.setTheme('light'); dropdownOpen = false"
              >
                <i class="bi bi-sun-fill"></i> Light
              </button>
            </li>
            <li>
              <button
                class="cmd-dropdown-item flex items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'dark' }"
                @click="themeStore.setTheme('dark'); dropdownOpen = false"
              >
                <i class="bi bi-moon-fill"></i> Dark
              </button>
            </li>
            <li>
              <button
                class="cmd-dropdown-item flex items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'high-contrast-light' }"
                @click="themeStore.setTheme('high-contrast-light'); dropdownOpen = false"
              >
                <i class="bi bi-circle-half"></i> High Contrast Light
              </button>
            </li>
            <li>
              <button
                class="cmd-dropdown-item flex items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'high-contrast-dark' }"
                @click="themeStore.setTheme('high-contrast-dark'); dropdownOpen = false"
              >
                <i class="bi bi-circle-fill"></i> High Contrast Dark
              </button>
            </li>
            <li>
              <button
                class="cmd-dropdown-item flex items-center gap-2 text-cmd-danger"
                :class="{ active: themeStore.currentTheme === 'dont-click-me' }"
                @click="themeStore.setTheme('dont-click-me'); dropdownOpen = false"
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
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import WebsocketStatus from '../components/WebsocketStatus.vue';
import NotificationCenter from '../components/NotificationCenter.vue';
import { useGridLayoutStore } from '@/stores/gridLayout';
import { useThemeStore } from '@/stores/theme';

export default defineComponent({
  name: 'NavBar',
  setup() {
    const gridLayoutStore = useGridLayoutStore();
    const themeStore = useThemeStore();
    const dropdownOpen = ref(false);

    const handleClickOutside = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      if (dropdownOpen.value && !target.closest('.dropdown')) {
        dropdownOpen.value = false;
      }
    };

    onMounted(() => {
      document.addEventListener('click', handleClickOutside);
    });

    onUnmounted(() => {
      document.removeEventListener('click', handleClickOutside);
    });

    return { gridLayoutStore, themeStore, dropdownOpen };
  },
  computed: {
    title(): string {
      return this.getTitleForRoute(this.$route.path)
    },
    showGridControls(): boolean {
      return this.$route.meta.hasGridLayout === true;
    },
    themeIcon(): string {
      const icons: Record<string, string> = {
        'light': 'bi bi-sun-fill',
        'dark': 'bi bi-moon-fill',
        'high-contrast-light': 'bi bi-circle-half',
        'high-contrast-dark': 'bi bi-circle-fill',
        'dont-click-me': 'bi bi-exclamation-triangle-fill'
      };
      return icons[this.themeStore.currentTheme] || 'bi bi-sun-fill';
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
        '/ScienceTask' : "Science Mission",
      };

      return routeTitles[path] || 'Unknown Path';
    },
  }
});
</script>

<style scoped>
.logo {
  left: 48%;
}

h1 {
  margin: 0 !important;
  font-size: clamp(1.25rem, 1rem + 0.5vw, 1.75rem) !important;
  line-height: 1 !important;
}

.nav-divider {
  height: clamp(32px, 2.5vw, 48px);
}

.theme-btn {
  min-width: 48px;
  height: 100%;
  padding: 0.25rem 0.5rem;
  cursor: pointer;
  background-color: var(--card-bg);
}

.theme-btn i {
  font-size: 1.25rem;
}
</style>
