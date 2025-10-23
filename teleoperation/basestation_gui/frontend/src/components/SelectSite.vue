<template>
  <!-- TODO: Add in conditonal buttons based on which buttons are clicked already -->
  <!-- we want to rotate the hex hub in one direction? -->
  <div class="w-100">
    <h2>Select Site</h2>
    <div class="form-check form-check-inline">
      <input
        v-model="site"
        class="form-check-input"
        type="radio"
        name="flexRadioDefault"
        id="site0"
        value="0"
      />
      <label class="form-check-label" for="site0"> Site A </label>
    </div>
    <div class="form-check form-check-inline">
      <input
        v-model="site"
        class="form-check-input"
        type="radio"
        name="flexRadioDefault"
        id="site1"
        value="1"
      />
      <label class="form-check-label" for="site1"> Site B </label>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, defineEmits } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

const emit = defineEmits(['site'])

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const site = ref(0)
const autoShutdownEnabled = ref(true)

const scienceMessage = computed(() => messages.value['science'])

watch(site, (event) => {
  emit('site', event)
})

watch(scienceMessage, (msg) => {
  if (msg && msg.type == 'auto_shutoff') {
    if (!msg.success) {
      autoShutdownEnabled.value = !autoShutdownEnabled.value
      alert('Toggling Auto Shutdown failed.')
    }
  }
})
</script>

<style scoped></style>
