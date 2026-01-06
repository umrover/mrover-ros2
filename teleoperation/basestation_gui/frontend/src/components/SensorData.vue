<template>
  <div class="d-flex flex-row gap-2 justify-content-between align-items-start pb-1">
    <h3>Sensor Data</h3>
    <div class="d-flex flex-row gap-2">
      <button class="btn btn-primary text-nowrap" @click="showModal = true">
        View All
      </button>
      <button class="btn btn-secondary text-nowrap" @click="downloadCSV">
        <i class="bi bi-download"></i> CSV
      </button>
    </div>
  </div>
  <div class="d-flex align-items-start">
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

  </div>

  <div class="d-flex flex-column gap-2 mt-2 flex-fill" style="min-height: 0">
    <div class="d-flex gap-2" style="flex: 1; min-height: 0">
      <div class="bg-white border rounded p-2 d-flex flex-column" style="flex: 1; min-width: 0">
        <div class="d-flex justify-content-between align-items-center mb-1">
          <strong style="font-size: 14px; color: #000000">Humidity (%)</strong>
        </div>
        <div style="flex: 1; min-height: 0">
          <canvas id="chart0"></canvas>
        </div>
      </div>
      <div class="bg-white border rounded p-2 d-flex flex-column" style="flex: 1; min-width: 0">
        <div class="d-flex justify-content-between align-items-center mb-1">
          <strong style="font-size: 14px; color: #000000">UV Index</strong>
        </div>
        <div style="flex: 1; min-height: 0">
          <canvas id="chart1"></canvas>
        </div>
      </div>
    </div>
    <div class="d-flex gap-2" style="flex: 1; min-height: 0">
      <div class="bg-white border rounded p-2 d-flex flex-column" style="flex: 1; min-width: 0">
        <div class="d-flex justify-content-between align-items-center mb-1">
          <strong style="font-size: 14px; color: #000000">Ozone (ppb)</strong>
        </div>
        <div style="flex: 1; min-height: 0">
          <canvas id="chart2"></canvas>
        </div>
      </div>
      <div class="bg-white border rounded p-2 d-flex flex-column" style="flex: 1; min-width: 0">
        <div class="d-flex justify-content-between align-items-center mb-1">
          <strong style="font-size: 14px; color: #000000">Pressure (Pa)</strong>
        </div>
        <div style="flex: 1; min-height: 0">
          <canvas id="chart3"></canvas>
        </div>
      </div>
    </div>
  </div>

  <SensorModal
    v-if="showModal"
    :sensor-history="sensor_history"
    :time-counter="timeCounter"
    @close="showModal = false"
    @reset="resetHistory"
  />
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import Chart from 'chart.js/auto'
import type { SensorData } from '../types/sensors'
import type { ScienceMessage } from '@/types/websocket'
import SensorModal from '@/components/SensorModal.vue'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const showModal = ref(false)

const sensor_data = ref<SensorData>({
  sp_oxygen: 0,
  sp_uv: 0,
  sp_humidity: 0,
  sp_temp: 0,
  sp_ozone: 0,
  sp_co2: 0,
  sp_pressure: 0,
})
const sensor_history = ref<number[][]>([
  Array(20).fill(0), // sp_oxygen
  Array(20).fill(0), // sp_humidity
  Array(20).fill(0), // sp_temp
  Array(20).fill(0), // sp_uv
  Array(20).fill(0), // sp_ozone
  Array(20).fill(0), // sp_co2
  Array(20).fill(0), // sp_pressure
])
const timeCounter = ref(0)

const scienceMessage = computed(() => messages.value['science'])

watch(scienceMessage, (msg) => {
  if (!msg) return
  const scienceMsg = msg as ScienceMessage;
  switch (scienceMsg.type) {
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
      sensor_data.value.sp_ozone = scienceMsg.ppb
      break
    case 'sp_co2':
      sensor_data.value.sp_co2 = scienceMsg.ppm
      break
    case 'sp_pressure':
      sensor_data.value.sp_pressure = scienceMsg.pressure
      break
  }
})

const resetHistory = () => {
  sensor_history.value = [
    Array(20).fill(0),
    Array(20).fill(0),
    Array(20).fill(0),
    Array(20).fill(0),
    Array(20).fill(0),
    Array(20).fill(0),
    Array(20).fill(0),
  ]
  timeCounter.value = 0
}

const downloadCSV = () => {
  let csv = 'Time,Oxygen,Humidity,Temp,UV,Ozone,CO2,Pressure\n'

  const firstHistory = sensor_history.value[0]
  if (!firstHistory) return

  const numRows = firstHistory.length
  for (let i = 0; i < numRows; ++i) {
    const row = sensor_history.value.map(sensor => sensor?.[i] ?? 0)
    csv += `${i},${row.join(',')}\n`
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

  const chartConfigs = [
    { title: 'Humidity (%)', datasets: [{ label: 'Humidity', color: '#3BB273', historyIndex: 1 }] },
    { title: 'UV Index', datasets: [{ label: 'UV', color: '#7768AE', historyIndex: 3 }] },
    { title: 'Ozone (ppb)', datasets: [{ label: 'Ozone', color: '#F9A825', historyIndex: 4 }] },
    { title: 'Pressure (Pa)', datasets: [{ label: 'Pressure', color: '#26A69A', historyIndex: 6 }] },
  ];

  const maxHistory = 20;

  for (let i = 0; i < chartConfigs.length; ++i) {
    waitForElm(`#chart${i}`).then(canvasElement => {
      if (canvasElement instanceof HTMLCanvasElement) {
        const config = chartConfigs[i];
        if (!config) return;

        const datasets = config.datasets.map(ds => ({
          label: ds.label,
          data: Array(20).fill(0),
          fill: false,
          borderColor: ds.color,
          tension: 0.1,
        }));

        charts[i] = new Chart(canvasElement, {
          type: 'line',
          data: {
            labels: Array.from({ length: 20 }, (_, i) => i),
            datasets: datasets,
          },
          options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            plugins: {
              title: {
                display: false
              },
              legend: {
                display: false
              }
            },
            scales: {
              y: {
                beginAtZero: false,
                ticks: {
                  font: { size: 10 },
                  maxTicksLimit: 8,
                  precision: 2,
                }
              },
              x: {
                ticks: { font: { size: 10 } }
              }
            },
          },
        });
      }
    });
  }

  // Set up the interval to update chart data
  setInterval(() => {
    sensor_history.value[0]?.push(sensor_data.value.sp_oxygen);
    sensor_history.value[1]?.push(sensor_data.value.sp_humidity);
    sensor_history.value[2]?.push(sensor_data.value.sp_temp);
    sensor_history.value[3]?.push(sensor_data.value.sp_uv);
    sensor_history.value[4]?.push(sensor_data.value.sp_ozone);
    sensor_history.value[5]?.push(sensor_data.value.sp_co2);
    sensor_history.value[6]?.push(sensor_data.value.sp_pressure);

    timeCounter.value++;

    for (let x = 0; x < 7; ++x) {
      const history = sensor_history.value[x];
      if (history && history.length > maxHistory) {
        history.shift();
      }
    }

    for (let i = 0; i < chartConfigs.length; ++i) {
      const chart = charts[i];
      const config = chartConfigs[i];
      if (chart && config) {
        config.datasets.forEach((ds, idx) => {
          const dataset = chart.data.datasets[idx];
          const historyData = sensor_history.value[ds.historyIndex];
          if (dataset && historyData) {
            dataset.data = [...historyData];
          }
        });

        const firstHistory = sensor_history.value[0];
        if (firstHistory) {
          const startTime = Math.max(0, timeCounter.value - firstHistory.length + 1);
          chart.data.labels = Array.from(
            { length: firstHistory.length },
            (_, i) => startTime + i
          );
        }
        chart.update();
      }
    }
  }, 1000);
})
</script>
