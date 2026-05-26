<template>
  <div class="flex gap-2 h-full min-h-0">
    <div class="flex flex-col gap-1 w-52 shrink-0">
      <h4 class="component-header">Sensor Data</h4>
      <div class="flex gap-1">
        <button class="btn btn-outline-control btn-sm flex-1" data-testid="pw-sensor-view-all" @click="showModal = true">View All</button>
        <button class="btn btn-outline-secondary btn-sm flex-1" data-testid="pw-sensor-csv-btn" @click="downloadCSV">
          <i class="bi bi-download"></i> CSV
        </button>
      </div>
      <div class="grid grid-cols-2 gap-1 flex-1 mt-1">
        <div class="flex flex-col items-center justify-center rounded p-1" :class="sensor_data.sensor_states.oxygen_state ? 'bg-theme-view' : 'bg-yellow-500/50'">
          <span class="font-semibold uppercase text-muted text-[0.6rem] tracking-[0.05em]">Oxygen</span>
          <span class="font-bold text-sm">{{ sensor_data.sp_oxygen.toFixed(2) }}<span class="text-muted ml-1 text-xs">%</span></span>
        </div>
        <div class="flex flex-col items-center justify-center rounded p-1" :class="sensor_data.sensor_states.uv_state ? 'bg-theme-view' : 'bg-yellow-500/50'">
          <span class="font-semibold uppercase text-muted text-[0.6rem] tracking-[0.05em]">UV</span>
          <span class="font-bold text-sm">{{ sensor_data.sp_uv.toFixed(2) }}<span class="text-muted ml-1 text-xs">idx</span></span>
        </div>
        <div class="flex flex-col items-center justify-center rounded p-1" :class="sensor_data.sensor_states.thp_state ? 'bg-theme-view' : 'bg-yellow-500/50'">
          <span class="font-semibold uppercase text-muted text-[0.6rem] tracking-[0.05em]">Humidity</span>
          <span class="font-bold text-sm">{{ sensor_data.sp_humidity.toFixed(2) }}<span class="text-muted ml-1 text-xs">%</span></span>
        </div>
        <div class="flex flex-col items-center justify-center rounded p-1" :class="sensor_data.sensor_states.thp_state ? 'bg-theme-view' : 'bg-yellow-500/50'">
          <span class="font-semibold uppercase text-muted text-[0.6rem] tracking-[0.05em]">Temp</span>
          <span class="font-bold text-sm">{{ sensor_data.sp_temp.toFixed(2) }}<span class="text-muted ml-1 text-xs">&deg;C</span></span>
        </div>
        <div class="flex flex-col items-center justify-center rounded p-1" :class="sensor_data.sensor_states.ozone_state ? 'bg-theme-view' : 'bg-yellow-500/50'">
          <span class="font-semibold uppercase text-muted text-[0.6rem] tracking-[0.05em]">Ozone</span>
          <span class="font-bold text-sm">{{ sensor_data.sp_ozone.toFixed(2) }}<span class="text-muted ml-1 text-xs">ppb</span></span>
        </div>
        <div class="flex flex-col items-center justify-center rounded p-1" :class="sensor_data.sensor_states.co2_state ? 'bg-theme-view' : 'bg-yellow-500/50'">
          <span class="font-semibold uppercase text-muted text-[0.6rem] tracking-[0.05em]">CO2</span>
          <span class="font-bold text-sm">{{ sensor_data.sp_co2.toFixed(2) }}<span class="text-muted ml-1 text-xs">ppm</span></span>
        </div>
        <div class="col-span-2 flex flex-col items-center justify-center rounded p-1" :class="sensor_data.sensor_states.thp_state ? 'bg-theme-view' : 'bg-yellow-500/50'">
          <span class="font-semibold uppercase text-muted text-[0.6rem] tracking-[0.05em]">Pressure</span>
          <span class="font-bold text-sm">{{ sensor_data.sp_pressure.toFixed(0) }}<span class="text-muted ml-1 text-xs">Pa</span></span>
        </div>
      </div>
    </div>

    <div class="flex flex-col gap-2 flex-1 min-h-0 min-w-0">
      <div class="flex gap-2 flex-1 min-h-0">
        <div class="panel flex flex-col flex-1 min-w-0 p-1!">
          <div class="flex items-center justify-between">
            <span class="data-label">Humidity (%)</span>
            <button class="btn btn-outline-secondary btn-xs" @click="downloadChartPNG(0)"><i class="bi bi-download"></i></button>
          </div>
          <div class="grow min-h-0">
            <canvas ref="chartRef0"></canvas>
          </div>
        </div>
        <div class="panel flex flex-col flex-1 min-w-0 p-1!">
          <div class="flex items-center justify-between">
            <span class="data-label">UV Index</span>
            <button class="btn btn-outline-secondary btn-xs" @click="downloadChartPNG(1)"><i class="bi bi-download"></i></button>
          </div>
          <div class="grow min-h-0">
            <canvas ref="chartRef1"></canvas>
          </div>
        </div>
      </div>
      <div class="flex gap-2 flex-1 min-h-0">
        <div class="panel flex flex-col flex-1 min-w-0 p-1!">
          <div class="flex items-center justify-between">
            <span class="data-label">Ozone (ppb)</span>
            <button class="btn btn-outline-secondary btn-xs" @click="downloadChartPNG(2)"><i class="bi bi-download"></i></button>
          </div>
          <div class="grow min-h-0">
            <canvas ref="chartRef2"></canvas>
          </div>
        </div>
        <div class="panel flex flex-col flex-1 min-w-0 p-1!">
          <div class="flex items-center justify-between">
            <span class="data-label">Pressure (Pa)</span>
            <button class="btn btn-outline-secondary btn-xs" @click="downloadChartPNG(3)"><i class="bi bi-download"></i></button>
          </div>
          <div class="grow min-h-0">
            <canvas ref="chartRef3"></canvas>
          </div>
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
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import Chart from 'chart.js/auto'
import type { SensorData } from '../types/sensors'
import type {
  SPOxygenMessage,
  SPUVMessage,
  SPTemperatureMessage,
  SPHumidityMessage,
  SPOzoneMessage,
  SPCO2Message,
  SPPressureMessage,
} from '@/types/websocket'
import SensorModal from '@/components/SensorModal.vue'
import { currentTimestamp } from '@/utils/formatNumber'

const websocketStore = useWebsocketStore()

const showModal = ref(false)

const sensor_data = ref<SensorData>({
  sp_oxygen: 0,
  sp_uv: 0,
  sp_humidity: 0,
  sp_temp: 0,
  sp_ozone: 0,
  sp_co2: 0,
  sp_pressure: 0,
  sensor_states: {
    uv_state: false,
    thp_state: false,
    oxygen_state: false,
    ozone_state: false,
    co2_state: false,
  },
})
const sensor_history = ref<number[][]>([
  Array(30).fill(0),
  Array(30).fill(0),
  Array(30).fill(0),
  Array(30).fill(0),
  Array(30).fill(0),
  Array(30).fill(0),
  Array(30).fill(0),
])
const timeCounter = ref(0)

websocketStore.onMessage<SPOxygenMessage>('science', 'sp_oxygen', (msg) => {
  sensor_data.value.sp_oxygen = msg.percent
})
websocketStore.onMessage<SPUVMessage>('science', 'sp_uv', (msg) => {
  sensor_data.value.sp_uv = msg.uv_index
})
websocketStore.onMessage<SPTemperatureMessage>('science', 'sp_temp', (msg) => {
  sensor_data.value.sp_temp = msg.temperature
})
websocketStore.onMessage<SPHumidityMessage>('science', 'sp_humidity', (msg) => {
  sensor_data.value.sp_humidity = msg.relative_humidity
})
websocketStore.onMessage<SPOzoneMessage>('science', 'sp_ozone', (msg) => {
  sensor_data.value.sp_ozone = msg.ppb
})
websocketStore.onMessage<SPCO2Message>('science', 'sp_co2', (msg) => {
  sensor_data.value.sp_co2 = msg.percent
})
websocketStore.onMessage<SPPressureMessage>('science', 'sp_pressure', (msg) => {
  sensor_data.value.sp_pressure = msg.pressure
})

const resetHistory = () => {
  sensor_history.value = [
    Array(30).fill(0),
    Array(30).fill(0),
    Array(30).fill(0),
    Array(30).fill(0),
    Array(30).fill(0),
    Array(30).fill(0),
    Array(30).fill(0),
  ]
  timeCounter.value = 0
}

const downloadChartPNG = (index: number) => {
  const chart = charts[index]
  const config = chartConfigs[index]
  if (!chart || !config) return
  const url = chart.canvas.toDataURL('image/png')
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = `${config.title.replace(/[^a-zA-Z0-9]+/g, '_').replace(/^_|_$/g, '')}_${currentTimestamp()}.png`
  anchor.click()
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

const maxHistory = 30

onMounted(() => {
  for (let i = 0; i < chartConfigs.length; ++i) {
    const canvasElement = chartRefs[i]?.value
    if (!canvasElement) continue
    const config = chartConfigs[i]
    if (!config) continue

    const datasets = config.datasets.map(ds => ({
      label: ds.label,
      data: Array(30).fill(0),
      fill: false,
      borderColor: ds.color,
      tension: 0.1,
    }))

    charts[i] = new Chart(canvasElement, {
      type: 'line',
      data: {
        labels: Array.from({ length: 30 }, (_, i) => i),
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
            title: {
              display: true,
              text: config.title,
              font: { size: 10 },
            },
            ticks: {
              font: { size: 10 },
              maxTicksLimit: 8,
              precision: 2,
            },
          },
          x: {
            title: {
              display: true,
              text: 'Time (s)',
              font: { size: 10 },
            },
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

<style scoped>
.btn-xs {
  padding: 0 0.3rem;
  font-size: 0.7rem;
  line-height: 1;
  height: 2em;
  display: inline-flex;
  align-items: center;
}

.btn-xs i {
  font-size: 0.7rem;
}
</style>
