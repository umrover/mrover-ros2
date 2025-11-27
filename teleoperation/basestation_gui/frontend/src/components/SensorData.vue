<template>
  <h3>Sensor Data</h3>
  <div class="sensors align-items-center">
    <table class="table table-bordered table-sm mx-3 mb-0" id="capture">
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
          <td>{{ sensor_data.oxygen.toFixed(2) }}</td>
          <td>{{ sensor_data.oxygen_var.toFixed(2) }}</td>
          <td>{{ sensor_data.uv.toFixed(2) }}</td>
          <td>{{ sensor_data.uv_var.toFixed(2) }}</td>
          <td>{{ sensor_data.humidity.toFixed(2) }}</td>
          <td>{{ sensor_data.humidity_var.toFixed(2) }}</td>
          <td>{{ sensor_data.temp.toFixed(2) }}</td>
          <td>{{ sensor_data.temp_var.toFixed(2) }}</td>
        </tr>
      </tbody>
    </table>

    <table class="table table-bordered table-sm mx-3 mb-0">
      <thead>
        <tr class="table-success">
          <th>SP Sensors</th>
          <th>Oxygen (%)</th>
          <th>UV (index)</th>
          <th>Humidity (%)</th>
          <th>Temp (°C)</th>
          <th>Ozone (ppb)</th>
          <th>CO2 (ppm)</th>
          <th>Pressure (Pa)</th>
        </tr>
      </thead>
      <tbody>
        <tr>
          <th class="table-secondary">Live Data</th>
          <td>{{ sensor_data.sp_oxygen.toFixed(2) }}</td>
          <td>{{ sensor_data.sp_uv.toFixed(2) }}</td>
          <td>{{ sensor_data.sp_humidity.toFixed(2) }}</td>
          <td>{{ sensor_data.sp_temp.toFixed(2) }}</td>
          <td>{{ sensor_data.sp_ozone.toFixed(2) }}</td>
          <td>{{ sensor_data.sp_co2.toFixed(2) }}</td>
          <td>{{ sensor_data.sp_pressure.toFixed(2) }}</td>
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
        style="width: 100%; height: 150px; background-color: white"
      ></canvas>
    </div>
    <div style="width: 50%; overflow-x: scroll; margin-top: 5px">
      <canvas
        id="chart1"
        style="width: 100%; height: 150px; background-color: white"
      ></canvas>
    </div>
  </div>

  <div style="display: flex; flex-direction: row; gap: 10px; margin-top: 5px">
    <div style="width: 50%; overflow-x: scroll">
      <canvas
        id="chart2"
        style="width: 100%; height: 150px; background-color: white"
      ></canvas>
    </div>
    <div style="width: 50%; overflow-x: scroll">
      <canvas
        id="chart3"
        style="width: 100%; height: 150px; background-color: white"
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
  sp_oxygen: 0,
  sp_uv: 0,
  sp_humidity: 0,
  sp_temp: 0,
  sp_ozone: 0,
  sp_co2: 0,
  sp_pressure: 0,
})
const sensor_history = ref<number[][]>([
  Array(10).fill(0), // oxygen
  Array(10).fill(0), // humidity
  Array(10).fill(0), // temperature
  Array(10).fill(0), // uv
])
const timeCounter = ref(0)

const scienceMessage = computed(() => messages.value['science'])

watch(scienceMessage, (msg) => {
  if (!msg) return
  const scienceMsg = msg as ScienceMessage;
  switch (scienceMsg.type) {
    case 'oxygen':
      sensor_data.value.oxygen = scienceMsg.percent
      break
    case 'uv':
      sensor_data.value.uv = scienceMsg.uv_index
      break
    case 'temperature':
      sensor_data.value.temp = scienceMsg.temperature
      break
    case 'humidity':
      sensor_data.value.humidity = scienceMsg.relative_humidity
      break
    case 'sp_oxygen':
      sensor_data.value.sp_oxygen = scienceMsg.percent
      break
    case 'sp_uv':
      sensor_data.value.sp_uv = scienceMsg.uv_index
      break
    case 'sp_temp':
      sensor_data.value.sp_temp = scienceMsg.temperature
      break
    case 'sp_humidity':
      sensor_data.value.sp_humidity = scienceMsg.relative_humidity
      break
    case 'sp_ozone':
      sensor_data.value.sp_ozone = scienceMsg.ozone
      break
    case 'sp_co2':
      sensor_data.value.sp_co2 = scienceMsg.co2
      break
    case 'sp_pressure':
      sensor_data.value.sp_pressure = scienceMsg.pressure
      break
  }
})

const download = () => {
  let csv = 'Oxygen,UV (index),Humidity,Temperature (C),SP Oxygen,SP UV,SP Humidity,SP Temp,SP Ozone,SP CO2,SP Pressure\n'

  const numRows = sensor_history.value[0].length
  for (let i = 0; i < numRows; ++i) {
    const row = sensor_history.value.map(sensor => sensor[i])
    csv += row.join(',') + ','
    csv += `${sensor_data.value.sp_oxygen},${sensor_data.value.sp_uv},${sensor_data.value.sp_humidity},${sensor_data.value.sp_temp},${sensor_data.value.sp_ozone},${sensor_data.value.sp_co2},${sensor_data.value.sp_pressure}\n`
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
          labels: Array.from({ length: 10 }, (_, i) => i), // Initialize with 0-9
          datasets: [
            {
              label: titles[i],
              data: Array(10).fill(0), // Initialize with 10 zeros
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
            animation: false,
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

    timeCounter.value++;

    for (let x = 0; x < 4; ++x) {
      if (sensor_history.value[x].length > maxHistory) {
        sensor_history.value[x].shift();
      }
    }

    for (let x = 0; x < 4; ++x) {
      const chart = charts[x];
      if (chart) {
        chart.data.datasets[0].data = [...sensor_history.value[x]];
        // Create rolling time labels
        const startTime = Math.max(0, timeCounter.value - sensor_history.value[x].length + 1);
        chart.data.labels = Array.from(
          { length: sensor_history.value[x].length },
          (_, i) => startTime + i
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
