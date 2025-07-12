<template>
	<div class="wrapper">
		<div class="ps-3 pe-2 py-2 d-flex justify-content-between align-items-center position-relative">
			<a class="logo" href="/"><img src="/mrover.png" alt="MRover" title="MRover" width="200" /></a>
      <h1>{{ title }}</h1>
      <div class="d-flex gap-1">
        <WebsocketStatus />
        <NetworkMonitor v-if="showMonitor" />
      </div>
		</div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import NetworkMonitor from '../components/NetworkMonitor.vue';
import WebsocketStatus from '../components/WebsocketStatus.vue';

export default defineComponent({
  name: 'NavBar',
  computed: {
    title(): string {
      // @ts-expect-error ts dumb
      return this.getTitleForRoute(this.$route.path)
    },
    showMonitor(): boolean {
      // @ts-expect-error ts dumb
      return this.showNetworkMonitor(this.$route.path)
    },
  },
	components: {
    NetworkMonitor,
    WebsocketStatus
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
      };

      return routeTitles[path] || 'Unknown Path';
    },
    showNetworkMonitor(path: string): boolean {
      return path === '/SATask' || path === '/ISHTask';
    }
  }
});
</script>

<style scoped>
h1 {
	font-family: 'Consolas', 'Courier New', 'DejaVu Sans Mono', monospace;
  font-weight: 400;
  font-size: 2.5rem;
	letter-spacing: -0.1rem;
  margin: 0;
  user-select: none;
}

.logo {
  position: absolute;
  left: 48%;
}
</style>