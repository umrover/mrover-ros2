<template>
	<div class="wrapper">
		<div class="py-3 px-3 header">
			<a class="logo" href="/"><img src="/mrover.png" alt="MRover" title="MRover" width="200" /></a>
			<h1>{{ getTitleForRoute($route.path) }}</h1>
      <NetworkMonitor v-if="showNetworkMonitor($route.path)" />
		</div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue';
import NetworkMonitor from '../components/NetworkMonitor.vue';

export default defineComponent({
  name: 'NavBar',
	components: {
    NetworkMonitor,
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

.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  position: relative;
}

h1 {
	font-family: 'Consolas', 'Courier New', 'DejaVu Sans Mono', monospace;
  font-weight: 400;
  font-size: 2.5rem;
	letter-spacing: -0.1rem;
  margin: 0;
  user-select: none;
}

</style>