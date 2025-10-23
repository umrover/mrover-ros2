<template>
  <div class="flex w-100">
    <h2>White LED</h2>
    <ToggleButton
      :current-state="siteEnabled[site]"
      :label-enable-text="'LED Site ' + String.fromCharCode(site + 65)"
      :label-disable-text="'LED Site ' + String.fromCharCode(site + 65)"
      @change="toggleLEDs()"
    />
    <!-- <LEDIndicator
          :connected="siteEnabled[site]"
          :name="'LED Status'"
          :show_name="true"
      /> -->
  </div>
</template>

<script lang="ts" setup>
import { ref, defineProps } from 'vue'
import ToggleButton from './ToggleButton.vue'
import { scienceAPI } from '@/utils/api'

const props = defineProps({
  site: {
    type: Number,
    required: true,
  },
})

const siteEnabled = ref([false, false])

const toggleLEDs = async () => {
  siteEnabled.value[props.site] = !siteEnabled.value[props.site]

  try {
    const siteName = props.site === 0 ? 'a' : 'b'
    await scienceAPI.setWhiteLEDs(siteName, siteEnabled.value[props.site])
  } catch (error) {
    console.error('Failed to toggle white LEDs:', error)
    // Revert state on error
    siteEnabled.value[props.site] = !siteEnabled.value[props.site]
  }
}
</script>
