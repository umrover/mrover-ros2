<template>
  <div class="wrapper bg-theme-card border-b border-theme">
    <div class="pl-4 pr-2 py-2 flex justify-between items-center relative flex-nowrap">
      <a class="logo absolute" href="/"><img src="/mrover.png" alt="MRover" width="200" /></a>
      <div class="flex items-center gap-2">
        <h1 class="text-theme-primary">{{ title }}</h1>
        <div class="dropdown flex relative">
          <button
            class="theme-btn-inline flex items-center justify-center border-2 border-theme rounded"
            data-testid="pw-theme-dropdown"
            @click="dropdownOpen = !dropdownOpen"
          >
            <i :class="themeIcon"></i>
          </button>
          <ul class="dropdown-menu" :class="{ show: dropdownOpen }">
            <li>
              <button
                class="dropdown-item flex items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'light' }"
                @click="themeStore.setTheme('light'); dropdownOpen = false"
              >
                <i class="bi bi-sun-fill"></i> Light
              </button>
            </li>
            <li>
              <button
                class="dropdown-item flex items-center gap-2"
                :class="{ active: themeStore.currentTheme === 'dark' }"
                @click="themeStore.setTheme('dark'); dropdownOpen = false"
              >
                <i class="bi bi-moon-fill"></i> Dark
              </button>
            </li>
          </ul>
        </div>
        <div v-if="showGridControls" class="flex items-center gap-1 border-2 border-theme rounded px-2 py-1">
          <button
            class="btn btn-sm btn-icon-sm"
            :class="gridLayoutStore.locked ? 'btn-danger' : 'btn-success'"
            data-testid="pw-grid-lock-btn"
            @click="gridLayoutStore.toggleLock()"
          >
            <i :class="gridLayoutStore.locked ? 'bi bi-lock-fill' : 'bi bi-unlock-fill'"></i>
          </button>
          <button
            class="btn btn-sm btn-secondary btn-icon-sm"
            data-testid="pw-grid-reset-btn"
            @click="gridLayoutStore.triggerReset()"
          >
            <i class="bi bi-arrow-counterclockwise"></i>
          </button>
          <span class="ml-1 text-sm text-muted">Grid</span>
        </div>
      </div>
      <div class="flex items-stretch gap-2">
        <div
          class="border-2 border-theme rounded px-2 flex flex-col justify-center font-mono text-sm"
          :class="{ 'jetson-high-latency': jetsonLatencyReceived && jetsonLatencyMs !== null && jetsonLatencyMs > 100 }"
        >
          <span class="text-muted font-semibold">jetson</span>
          <span :class="jetsonLatencyReceived && jetsonLatencyMs === null ? 'text-danger' : ''">
            <span v-html="formatNumber(jetsonLatencyMs, 4, 0)"></span><span :class="jetsonLatencyReceived && jetsonLatencyMs === null ? '' : 'text-muted'">ms</span>
          </span>
        </div>
        <div class="border-l border-2 border-start-theme self-center nav-divider"></div>
        <WebsocketStatus />
        <div class="border-l border-2 border-start-theme self-center nav-divider"></div>
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
import { useWebsocketStore } from '@/stores/websocket';
import { formatNumber } from '@/utils/formatNumber';

export default defineComponent({
  name: 'NavBar',
  setup() {
    const gridLayoutStore = useGridLayoutStore();
    const themeStore = useThemeStore();
    const dropdownOpen = ref(false);
    const jetsonLatencyMs = ref<number | null>(null);
    const jetsonLatencyReceived = ref(false);

    const websocketStore = useWebsocketStore();
    websocketStore.setupWebSocket('latency');
    websocketStore.onMessage<{ type: string; latency_ms: number | null }>('latency', 'jetson_ping', (msg) => {
      jetsonLatencyMs.value = msg.latency_ms;
      jetsonLatencyReceived.value = true;
    });

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
      websocketStore.closeWebSocket('latency');
    });

    return { gridLayoutStore, themeStore, dropdownOpen, jetsonLatencyMs, jetsonLatencyReceived, formatNumber };
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
        'dark': 'bi bi-moon-fill'
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
  margin: 0;
  font-size: clamp(1.25rem, 1rem + 0.5vw, 1.75rem);
  font-weight: 600;
  line-height: 1;
}

.nav-divider {
  height: clamp(32px, 2.5vw, 48px);
}

.theme-btn-inline {
  width: 28px;
  height: 28px;
  padding: 0;
  cursor: pointer;
  background-color: var(--card-bg);
  flex-shrink: 0;
}

.theme-btn-inline i {
  font-size: 0.85rem;
}

@keyframes jetson-flash-red {
  0%, 100% { background-color: transparent; }
  50% { background-color: rgba(220, 53, 69, 0.4); }
}

.jetson-high-latency {
  animation: jetson-flash-red 1s ease-in-out infinite;
}
</style>
