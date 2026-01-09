<template>
	<div class="wrapper">
		<div class="ps-3 pe-2 py-2 d-flex justify-content-between align-items-center position-relative">
			<a class="logo" href="/"><img src="/mrover.png" alt="MRover" title="MRover" width="200" /></a>
      <div class="d-flex align-items-center gap-3">
        <h1>{{ title }}</h1>
        <div v-if="showGridControls" class="d-flex align-items-center gap-1 border rounded px-2 py-1">
          <button
            class="btn btn-sm px-2"
            :class="gridLayoutStore.locked ? 'btn-danger' : 'btn-success'"
            @click="gridLayoutStore.toggleLock()"
          >
            <i :class="gridLayoutStore.locked ? 'bi bi-lock-fill' : 'bi bi-unlock-fill'"></i>
          </button>
          <button
            class="btn btn-sm btn-secondary px-2"
            @click="gridLayoutStore.triggerReset()"
          >
            <i class="bi bi-arrow-counterclockwise"></i>
          </button>
          <span class="ms-1 small text-muted">Grid</span>
        </div>
      </div>
      <div class="d-flex align-items-center gap-3">
        <WebsocketStatus />
        <div class="border-start border-2" style="height: 40px;"></div>
        <NotificationCenter />
      </div>
		</div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import WebsocketStatus from '../components/WebsocketStatus.vue';
import NotificationCenter from '../components/NotificationCenter.vue';
import { useGridLayoutStore } from '@/stores/gridLayout';

const GRID_LAYOUT_ROUTES = ['/AutonTask', '/ScienceTask', '/ESTask', '/DMTask'];

export default defineComponent({
  name: 'NavBar',
  setup() {
    const gridLayoutStore = useGridLayoutStore();
    return { gridLayoutStore };
  },
  computed: {
    title(): string {
      return this.getTitleForRoute(this.$route.path)
    },
    showGridControls(): boolean {
      return GRID_LAYOUT_ROUTES.includes(this.$route.path);
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
  position: absolute;
  left: 48%;
}
</style>