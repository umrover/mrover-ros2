<template>
  <div class="flex items-center gap-2 mb-2">
    <h4 class="component-header">Sensor<br>Data</h4>
    <div class="flex flex-col gap-1">
      <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm" data-testid="pw-sensor-view-all" @click="showModal = true">
        View All
      </button>
      <button class="cmd-btn cmd-btn-outline-secondary cmd-btn-sm" data-testid="pw-sensor-csv-btn" @click="downloadCSV">
        <i class="bi bi-download"></i> CSV
      </button>
    </div>
    <div class="flex gap-2 grow">
      <div class="flex-1 flex flex-col items-center bg-theme-view rounded p-1">
        <span class="font-semibold uppercase text-muted text-[0.65rem] tracking-[0.05em]">Oxygen</span>
        <span class="font-bold">{{ sensor_data.sp_oxygen.toFixed(2) }}<span class="text-muted ml-1 text-sm">%</span></span>
      </div>
      <div class="flex-1 flex flex-col items-center bg-theme-view rounded p-1">
        <span class="font-semibold uppercase text-muted text-[0.65rem] tracking-[0.05em]">UV</span>
        <span class="font-bold">{{ sensor_data.sp_uv.toFixed(2) }}<span class="text-muted ml-1 text-sm">idx</span></span>
      </div>
      <div class="flex-1 flex flex-col items-center bg-theme-view rounded p-1">
        <span class="font-semibold uppercase text-muted text-[0.65rem] tracking-[0.05em]">Humidity</span>
        <span class="font-bold">{{ sensor_data.sp_humidity.toFixed(2) }}<span class="text-muted ml-1 text-sm">%</span></span>
      </div>
      <div class="flex-1 flex flex-col items-center bg-theme-view rounded p-1">
        <span class="font-semibold uppercase text-muted text-[0.65rem] tracking-[0.05em]">Temp</span>
        <span class="font-bold">{{ sensor_data.sp_temp.toFixed(2) }}<span class="text-muted ml-1 text-sm">&deg;C</span></span>
      </div>
      <div class="flex-1 flex flex-col items-center bg-theme-view rounded p-1">
        <span class="font-semibold uppercase text-muted text-[0.65rem] tracking-[0.05em]">Ozone</span>
        <span class="font-bold">{{ sensor_data.sp_ozone.toFixed(2) }}<span class="text-muted ml-1 text-sm">ppb</span></span>
      </div>
      <div class="flex-1 flex flex-col items-center bg-theme-view rounded p-1">
        <span class="font-semibold uppercase text-muted text-[0.65rem] tracking-[0.05em]">CO2</span>
        <span class="font-bold">{{ sensor_data.sp_co2.toFixed(2) }}<span class="text-muted ml-1 text-sm">ppm</span></span>
      </div>
      <div class="flex-1 flex flex-col items-center bg-theme-view rounded p-1">
        <span class="font-semibold uppercase text-muted text-[0.65rem] tracking-[0.05em]">Pressure</span>
        <span class="font-bold">{{ sensor_data.sp_pressure.toFixed(0) }}<span class="text-muted ml-1 text-sm">Pa</span></span>
      </div>
    </div>
  </div>

  <div class="flex flex-col gap-2 grow min-h-0">
    <div class="flex gap-2 flex-1 min-h-0">
      <div class="cmd-panel flex flex-col flex-1 min-w-0 p-1">
        <span class="cmd-data-label mb-1">Humidity (%)</span>
        <div class="grow min-h-0">
          <canvas ref="chartRef0"></canvas>
        </div>
      </div>
      <div class="cmd-panel flex flex-col flex-1 min-w-0 p-1">
        <span class="cmd-data-label mb-1">UV Index</span>
        <div class="grow min-h-0">
          <canvas ref="chartRef1"></canvas>
        </div>
      </div>
    </div>
    <div class="flex gap-2 flex-1 min-h-0">
      <div class="cmd-panel flex flex-col flex-1 min-w-0 p-1">
        <span class="cmd-data-label mb-1">Ozone (ppb)</span>
        <div class="grow min-h-0">
          <canvas ref="chartRef2"></canvas>
        </div>
      </div>
      <div class="cmd-panel flex flex-col flex-1 min-w-0 p-1">
        <span class="cmd-data-label mb-1">Pressure (Pa)</span>
        <div class="grow min-h-0">
          <canvas ref="chartRef3"></canvas>
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
import { ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'
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
  Array(20).fill(0),
  Array(20).fill(0),
  Array(20).fill(0),
  Array(20).fill(0),
  Array(20).fill(0),
  Array(20).fill(0),
  Array(20).fill(0),
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

const chartRef0 = ref<HTMLCanvasElement | null>(null)
const chartRef1 = ref<HTMLCanvasElement | null>(null)
const chartRef2 = ref<HTMLCanvasElement | null>(null)
const chartRef3 = ref<HTMLCanvasElement | null>(null)
const chartRefs = [chartRef0, chartRef1, chartRef2, chartRef3]

const charts: Chart[] = []
let updateInterval: number | undefined = undefined

const chartConfigs = [
  { title: 'Humidity (%)', datasets: [{ label: 'Humidity', color: '#3BB273', historyIndex: 1 }] },
  { title: 'UV Index', datasets: [{ label: 'UV', color: '#7768AE', historyIndex: 3 }] },
  { title: 'Ozone (ppb)', datasets: [{ label: 'Ozone', color: '#F9A825', historyIndex: 4 }] },
  { title: 'Pressure (Pa)', datasets: [{ label: 'Pressure', color: '#26A69A', historyIndex: 6 }] },
]

const maxHistory = 20

onMounted(() => {
  for (let i = 0; i < chartConfigs.length; ++i) {
    const canvasElement = chartRefs[i]?.value
    if (!canvasElement) continue
    const config = chartConfigs[i]
    if (!config) continue

    const datasets = config.datasets.map(ds => ({
      label: ds.label,
      data: Array(20).fill(0),
      fill: false,
      borderColor: ds.color,
      tension: 0.1,
    }))

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
          title: { display: false },
          legend: { display: false },
        },
        scales: {
          y: {
            beginAtZero: false,
            ticks: {
              font: { size: 10 },
              maxTicksLimit: 8,
              precision: 2,
            },
          },
          x: {
            ticks: { font: { size: 10 } },
          },
        },
      },
    })
  }

  updateInterval = window.setInterval(() => {
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

onBeforeUnmount(() => {
  if (updateInterval !== undefined) {
    clearInterval(updateInterval)
    updateInterval = undefined
  }
  for (const chart of charts) {
    chart?.destroy()
  }
  charts.length = 0
})
</script>


