<template>
  <div class="wrap">
    <h3 v-if="isNinhydrin">Ninhydrin Test Controls</h3>
    <h3 v-else>Benedict's Test Controls</h3>
    <div class="box1 heaters">
      <ToggleButton
        id="heater"
        :current-state="heaters[site].enabled"
        :label-enable-text="'Heater ' + String.fromCharCode(65 + site)"
        :label-disable-text="'Heater ' + String.fromCharCode(65 + site)"
        @change="toggleHeater()"
      />
      <p :style="{ color: heaters[site].color }">
        Thermistor {{ String.fromCharCode(65 + site) }}:
        {{ heaters[site].temp.toFixed(2) }} C°
      </p>
    </div>
    <div class="comms heaterStatus">
      <LEDIndicator
        :connected="heaters[site].state"
        :name="'Heater ' + String.fromCharCode(65 + site) + ' Status'"
        :show_name="true"
      />
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, defineProps } from 'vue'
import ToggleButton from './ToggleButton.vue'
import LEDIndicator from './LEDIndicator.vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import { scienceAPI } from '@/utils/api'

const props = defineProps({
  site: {
    type: Number,
    required: true,
  },
  isNinhydrin: {
    //true = Ninhydrin, false = benedict's
    type: Boolean,
    required: true,
  },
})

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const heaters = ref([
  {
    enabled: false,
    temp: 0,
    state: false,
    color: 'grey',
  },
  {
    enabled: false,
    temp: 0,
    state: false,
    color: 'grey',
  },
])

const scienceMessage = computed(() => messages.value['science'])

watch(scienceMessage, (msg) => {
  if (msg.type == 'thermistors') {
    if (props.isNinhydrin) {
      heaters.value[props.site].temp =
        msg.temps[props.site * 2 + 1].temperature
    } else {
      heaters.value[props.site].temp = msg.temps[props.site * 2].temperature
    }
  } else if (msg.type == 'heater_states') {
    if (props.isNinhydrin) {
      heaters.value[props.site].state = msg.state[props.site * 2 + 1]
      heaters.value[props.site].enabled = heaters.value[props.site].state
    } else {
      heaters.value[props.site].state = msg.state[props.site * 2]
      heaters.value[props.site].enabled = heaters.value[props.site].state
    }
  }
})

const toggleHeater = () => {
  heaters.value[props.site].enabled = !heaters.value[props.site].enabled
  sendHeaterRequest()
}

const sendHeaterRequest = async () => {
  let heaterName = String.fromCharCode(props.site + 97)
  if (props.isNinhydrin) {
    heaterName += '1'
  } else heaterName += '0'

  try {
    await scienceAPI.setHeater(heaterName, heaters.value[props.site].enabled)
  } catch (error) {
    console.error('Failed to toggle heater:', error)
    // Revert state on error
    heaters.value[props.site].enabled = !heaters.value[props.site].enabled
  }
}
</script>

<style scoped></style>
