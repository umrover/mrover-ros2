<template>
  <h3>Sensor Data</h3>
  <div class="sensors align-items-center">
    <table class="table table-bordered mx-3 mb-0" id="capture">
      <thead>
        <tr class="table-primary">
          <th></th>
          <th colspan="2">Oxygen (%)</th>
          <th colspan="2">UV (index)</th>
          <th colspan="2">Humidity (%)</th>
          <th colspan="2">Temp (°C)</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <th class="table-secondary">
            Site {{ String.fromCharCode(site + 65) }}
          </th>
          <!-- entries assumes string, any type pair -->
          <td
            v-for="([, val], index) in Object.entries(sensor_data)"
            :key="index"
          >
            {{ val.toFixed(2) }}
          </td>
        </tr>
      </tbody>
    </table>

    <button class="btn btn-secondary" @click="download()">
      Save Data to CSV
    </button>
  </div>

  <div style="display: flex; flex-direction: row; gap: 10px">
    <div style="width: 50%; overflow-x: scroll">
      <canvas
        id="chart0"
        style="width: 100%; height: 200px; background-color: white"
      ></canvas>
    </div>
    <div style="width: 50%; overflow-x: scroll; margin-top: 5px">
      <canvas
        id="chart1"
        style="width: 100%; height: 200px; background-color: white"
      ></canvas>
    </div>
  </div>

  <div style="display: flex; flex-direction: row; gap: 10px; margin-top: 5px">
    <div style="width: 50%; overflow-x: scroll">
      <canvas
        id="chart2"
        style="width: 100%; height: 200px; background-color: white"
      ></canvas>
    </div>
    <div style="width: 50%; overflow-x: scroll">
      <canvas
        id="chart3"
        style="width: 100%; height: 200px; background-color: white"
      ></canvas>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import Chart from 'chart.js/auto'
import type { SensorData } from '../types/sensors'
import type { ScienceMessage } from '@/types/websocket'

defineProps<{
  site: number
}>()

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const sensor_data = ref<SensorData>({
  oxygen: 0,
  oxygen_var: 0,
  uv: 0,
  uv_var: 0,
  humidity: 0,
  humidity_var: 0,
  temp: 0,
  temp_var: 0,
})
const sensor_history = ref<number[][]>([
  [], // oxygen
  [], // humidity
  [], // temperature
  [], // uv
])

const scienceMessage = computed(() => messages.value['science'])

watch(scienceMessage, (msg) => {
  if (!msg) return
  const scienceMsg = msg as ScienceMessage;
  switch (scienceMsg.type) {
    case 'oxygen':
      sensor_data.value.oxygen = scienceMsg.percent
      // sensor_data.value.oxygen_var = msg.varianace
      break
    case 'uv':
      sensor_data.value.uv = scienceMsg.uv_index
      // sensor_data.value.uv_var = msg.varianace
      break
    case 'temperature':
      sensor_data.value.temp = scienceMsg.temperature
      // sensor_data.value.temp_var = msg.variance
      break
    case 'humidity':
      sensor_data.value.humidity = scienceMsg.relative_humidity
      // sensor_data.value.humidity_var = 100* msg.variance
      break
  }
})

const download = () => {
  // downloads csv of table
  let csv = 'Oxygen, UV (index), Humidity, Temperature (C)\n'

  const numRows = sensor_history.value[0].length // transpose (flip) array
  for (let i = 0; i < numRows; ++i) {
    const row = sensor_history.value.map(sensor => sensor[i])
    csv += row.join(',') + '\n'
  }

  const anchor = document.createElement('a')
  anchor.href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv)
  anchor.download = 'sensor_data.csv'
  anchor.click()
}

onMounted(() => {
  const charts: Chart[] = [];

  // This helper function waits for a DOM element to be available
  function waitForElm(selector: string): Promise<Element | null> {
    return new Promise(resolve => {
      const elm = document.querySelector(selector);
      if (elm) {
        return resolve(elm);
      }

      const observer = new MutationObserver(() => {
        const elm = document.querySelector(selector);
        if (elm) {
          observer.disconnect();
          resolve(elm);
        }
      });

      observer.observe(document.body, {
        childList: true,
        subtree: true,
      });
    });
  }

  const titles = [
    'Oxygen Percentage Over Time (s)',
    'Relative Humidity Over Time (s)',
    'Temperature (C) Over Time (s)',
    'UV Index Over Time (s)',
  ];

  const lineColors = ['#4D9DE0', '#E15554', '#3BB273', '#7768AE'];
  const maxHistory = 10;

  // Create the four charts
  for (let i = 0; i < 4; ++i) {
    waitForElm(`#chart${i}`).then(canvasElement => {
      // Ensure the element exists and is a canvas before creating the chart
      if (canvasElement instanceof HTMLCanvasElement) {
        const data = {
          labels: [] as number[], // Initialize labels array
          datasets: [
            {
              label: titles[i],
              data: [] as number[], // ✅ Plain array, not reactive
              fill: false,
              borderColor: lineColors[i],
              tension: 0.1,
            },
          ],
        };

        charts[i] = new Chart(canvasElement, {
          type: 'line',
          data: data,
          options: {
            responsive: false,
            scales: {
              y: {
                beginAtZero: true,
              },
            },
          },
        });
      }
    });
  }

  // Set up the interval to update chart data
  setInterval(() => {
    sensor_history.value[0].push(sensor_data.value.oxygen);
    sensor_history.value[1].push(sensor_data.value.humidity);
    sensor_history.value[2].push(sensor_data.value.temp);
    sensor_history.value[3].push(sensor_data.value.uv);

    for (let x = 0; x < 4; ++x) {
      if (sensor_history.value[x].length > maxHistory) {
        sensor_history.value[x].shift();
      }
    }

    for (let x = 0; x < 4; ++x) {
      const chart = charts[x];
      if (chart) {
        chart.data.datasets[0].data = [...sensor_history.value[x]];
        // Update the labels to match the data length
        chart.data.labels = Array.from(
          { length: sensor_history.value[x].length },
          (_, i) => i
        );
        chart.update();
      }
    }
  }, 1000);
})
</script>

<style scoped>
.sensors {
  display: flex;
  align-items: start;
}
</style>
