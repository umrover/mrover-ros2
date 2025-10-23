<template>
  <div class="wrap box">
    <div class="d-flex justify-content-between align-middle">
      <h3>Soil Data</h3>
      <button class="btn btn-primary" @click="download()">
        Save to CSV
      </button>
    </div>
    <div class="table-responsive">
      <table class="table" id="capture">
        <thead>
          <tr class="table-primary">
            <th scope="col">Temperature</th>
          </tr>
        </thead>
        <tbody>
          <tr class="bold-border">
            <td>{{ temp.toFixed(2) }}ºC</td>
          </tr>
        </tbody>
        <thead>
          <tr class="table-primary">
            <th scope="col">Temperature</th>
          </tr>
        </thead>
        <tbody>
          <tr class="bold-border">
            <td>{{ temp.toFixed(2) }}ºC</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, defineProps } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import html2canvas from 'html2canvas'

const props = defineProps({
  site: {
    type: Number,
    required: true,
  },
})

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const temp = ref(0)
const humidity = ref(0)
const tempArray = ref<number[]>([])
const timestamps = ref<number[]>([])
const readData = ref(false)
const prevState = ref(false)
const exponents = ref<number[]>([])
const predictedTemp = ref<number | null>(null)

const scienceMessage = computed(() => messages.value['science'])

watch(scienceMessage, (msg) => {
  if (!msg) return
  switch (msg.type) {
    case 'soil_temp':
      temp.value = msg.temperature
      if (readData.value) {
        tempArray.value.push(temp.value)
        timestamps.value.push(Date.now())
      } else {
        predictedTemp.value = predictTemp(Date.now())
      }
      break
    case 'soil_humidity':
      humidity.value = msg.relative_humidity
      break
    case 'poly_fit':
      exponents.value = msg.exponents
      break
  }
})

watch(readData, () => {
  if (!readData.value) {
    // TODO: publishPolyfit - backend handler not implemented
    // publishPolyfit()
  } else if (readData.value && readData.value != prevState.value) {
    exponents.value = []
    tempArray.value = []
    timestamps.value = []
  }
  prevState.value = readData.value
})

const predictTemp = (timestamp: number) => {
  if (!exponents.value || exponents.value.length < 2) return 0
  const val = exponents.value[0] * timestamp + exponents.value[1]
  return Math.exp(val)
}

const download = () => {
  // downloads screenshot of table
  const table = document.querySelector('#capture') as HTMLElement
  html2canvas(table)
    .then(canvas => {
      canvas.style.display = 'none'
      document.body.appendChild(canvas)
      return canvas
    })
    .then(canvas => {
      const image = canvas.toDataURL('image/png')
      const a = document.createElement('a')
      a.setAttribute(
        'download',
        'sensor_data_site_' +
          String.fromCharCode(props.site + 65) +
          '_' +
          new Date(Date.now()).toString() +
          '.png',
      )
      a.setAttribute('href', image)
      a.click()
      canvas.remove()
    })
}
</script>
