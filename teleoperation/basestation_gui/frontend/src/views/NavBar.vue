<template>
	<div class="wrapper">
		<div class="ps-3 pe-2 py-2 d-flex justify-content-between align-items-center position-relative">
			<a class="logo" href="/"><img src="/mrover.png" alt="MRover" title="MRover" width="200" /></a>
      <h1>{{ title }}</h1>
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

export default defineComponent({
  name: 'NavBar',
  computed: {
    title(): string {
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
        '/Cameras': 'Camera View',
        '/SPTask' : "Science Payload",
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
</style>