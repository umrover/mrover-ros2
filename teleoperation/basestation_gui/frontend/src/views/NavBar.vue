<template>
	<div class="wrapper">
		<div class="shadow py-3 px-4 header">
			<a class="logo" href="/"><img src="/mrover.png" alt="MRover" title="MRover" width="200" /></a>
			<h1>{{ getTitleForRoute($route.path) }}</h1>
			<NetworkMonitor />
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
        '/ESTask': 'Equipment Servicing Mission',
        '/ISHTask': 'ISH Mission',
        '/SATask': 'Sample Acquisition Mission',
        '/Cameras': 'Camera View',
      };

      return routeTitles[path] || 'Unknown Path';
    }
  }
});
</script>

<style scoped>

.wrapper {
	background-color: #dddddd;
}

h1 {
  font-weight: 400;
  font-size: 3rem;
  margin: 0;
  user-select: none;
}

</style>