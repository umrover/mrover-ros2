<template>
	<div class="wrapper">
		<p class="stat px-2">TX: {{ tx.toFixed(2) }} </p>
		<p class="stat px-2">RX: {{ rx.toFixed(2) }} </p>
	</div>
</template>

<script lang="ts">
import Vuex from 'vuex';
const { mapState } = Vuex;

export default {
	data() {
		return {
			tx: 0,
			rx: 0
		};
	},

	computed: {
	...mapState('websocket', ['message'])
	},

	watch: {
		message(msg) {
			if(msg.type == "network_monitor") {
				this.tx = msg.tx;
				this.rx = msg.rx;
			}
		}
	}
};
</script>
  
<style scoped>
.wrapper {
  display: flex;
  flex-direction: row;
  gap: 0;
  font-weight: 800;
  font-size: 1.2rem;
  color: #333;
  background: rgba(220, 220, 220, 1);
  padding: 4px 6px;
  border-radius: 8px;
  width: fit-content;
  box-shadow: 0 2px 5px rgba(0,0,0,0.1);
  user-select: none;
}

.stat {
  margin: 0;
}
</style>
