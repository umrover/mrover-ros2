<template>
  <div class="d-flex align-items-center gap-2 mb-2">
    <h4 class="component-header">Sensor<br>Data</h4>
    <div class="d-flex flex-column gap-1">
      <button class="btn btn-outline-control btn-sm border-2" @click="showModal = true">
        View All
      </button>
      <button class="btn btn-outline-secondary btn-sm border-2" @click="downloadCSV">
        <i class="bi bi-download"></i> CSV
      </button>
    </div>
    <div class="d-flex gap-2 flex-grow-1">
      <div class="flex-fill d-flex flex-column align-items-center bg-theme-view rounded p-1">
        <span class="small fw-semibold text-uppercase text-muted sensor-label">Oxygen</span>
        <span class="fw-bold">{{ sensor_data.sp_oxygen.toFixed(2) }}<span class="text-muted ms-1 small">%</span></span>
      </div>
      <div class="flex-fill d-flex flex-column align-items-center bg-theme-view rounded p-1">
        <span class="small fw-semibold text-uppercase text-muted sensor-label">UV</span>
        <span class="fw-bold">{{ sensor_data.sp_uv.toFixed(2) }}<span class="text-muted ms-1 small">idx</span></span>
      </div>
      <div class="flex-fill d-flex flex-column align-items-center bg-theme-view rounded p-1">
        <span class="small fw-semibold text-uppercase text-muted sensor-label">Humidity</span>
        <span class="fw-bold">{{ sensor_data.sp_humidity.toFixed(2) }}<span class="text-muted ms-1 small">%</span></span>
      </div>
      <div class="flex-fill d-flex flex-column align-items-center bg-theme-view rounded p-1">
        <span class="small fw-semibold text-uppercase text-muted sensor-label">Temp</span>
        <span class="fw-bold">{{ sensor_data.sp_temp.toFixed(2) }}<span class="text-muted ms-1 small">&deg;C</span></span>
      </div>
      <div class="flex-fill d-flex flex-column align-items-center bg-theme-view rounded p-1">
        <span class="small fw-semibold text-uppercase text-muted sensor-label">Ozone</span>
        <span class="fw-bold">{{ sensor_data.sp_ozone.toFixed(2) }}<span class="text-muted ms-1 small">ppb</span></span>
      </div>
      <div class="flex-fill d-flex flex-column align-items-center bg-theme-view rounded p-1">
        <span class="small fw-semibold text-uppercase text-muted sensor-label">CO2</span>
        <span class="fw-bold">{{ sensor_data.sp_co2.toFixed(2) }}<span class="text-muted ms-1 small">ppm</span></span>
      </div>
      <div class="flex-fill d-flex flex-column align-items-center bg-theme-view rounded p-1">
        <span class="small fw-semibold text-uppercase text-muted sensor-label">Pressure</span>
        <span class="fw-bold">{{ sensor_data.sp_pressure.toFixed(0) }}<span class="text-muted ms-1 small">Pa</span></span>
      </div>
    </div>
  </div>

  <div class="d-flex flex-column gap-2 flex-grow-1 min-height-0">
    <div class="d-flex gap-2 flex-fill min-height-0 flex-basis-0">
      <div class="cmd-panel d-flex flex-column flex-fill min-width-0 p-1 flex-basis-0">
        <span class="cmd-data-label mb-1">Humidity (%)</span>
        <div class="flex-grow-1 min-height-0">
          <canvas id="chart0"></canvas>
        </div>
      </div>
      <div class="cmd-panel d-flex flex-column flex-fill min-width-0 p-1 flex-basis-0">
        <span class="cmd-data-label mb-1">UV Index</span>
        <div class="flex-grow-1 min-height-0">
          <canvas id="chart1"></canvas>
        </div>
      </div>
    </div>
    <div class="d-flex gap-2 flex-fill min-height-0 flex-basis-0">
      <div class="cmd-panel d-flex flex-column flex-fill min-width-0 p-1 flex-basis-0">
        <span class="cmd-data-label mb-1">Ozone (ppb)</span>
        <div class="flex-grow-1 min-height-0">
          <canvas id="chart2"></canvas>
        </div>
      </div>
      <div class="cmd-panel d-flex flex-column flex-fill min-width-0 p-1 flex-basis-0">
        <span class="cmd-data-label mb-1">Pressure (Pa)</span>
        <div class="flex-grow-1 min-height-0">
          <canvas id="chart3"></canvas>
        </div>
      </div>
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

<script lang="ts">
import Vuex from 'vuex'
const { mapState } = Vuex
import Chart from 'chart.js/auto'
import type { SensorData } from '../types/sensors'
import type { WebSocketState } from '../types/websocket'

export default {
  props: {
    site: {
      type: Number,
      required: true,
    },
  },

  data(): {
    sensor_data: SensorData;
    sensor_history: number[][];
  } {
    return {
      sensor_data: {
        oxygen: 0,
        oxygen_var: 0,
        uv: 0,
        uv_var: 0,
        humidity: 0,
        humidity_var: 0,
        temp: 0,
        temp_var: 0,
      },
      sensor_history: [
        [], // oxygen
        [], // humidity
        [], // temperature
        [], // uv
      ],
    };
  },

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
                data: this.sensor_history[i],
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
      this.sensor_history[0].push(this.sensor_data.oxygen);
      this.sensor_history[1].push(this.sensor_data.humidity);
      this.sensor_history[2].push(this.sensor_data.temp);
      this.sensor_history[3].push(this.sensor_data.uv);

      for (let x = 0; x < 4; ++x) {
        if (this.sensor_history[x].length > maxHistory) {
          this.sensor_history[x].shift();
        }
      }

      for (let x = 0; x < 4; ++x) {
        const chart = charts[x];
        if (chart) {
          // Update the labels to match the data length
          chart.data.labels = Array.from(
            { length: this.sensor_history[x].length },
            (_, i) => i
          );
          chart.update();
        }
      }
    }, 1000);
  },

  computed: {
    ...mapState('websocket', {
      scienceMessage: (state: WebSocketState) => state.messages['science']
    }),
    sensorValues() {
      return [
        this.sensor_data.oxygen,
        this.sensor_data.oxygen_var,
        this.sensor_data.uv,
        this.sensor_data.uv_var,
        this.sensor_data.humidity,
        this.sensor_data.humidity_var,
        this.sensor_data.temp,
        this.sensor_data.temp_var,
      ]
    },
  },
  created() {
    // window.setInterval();
    // this.interval = window.setInterval(() => {
    //   this.randomizeSensorData();
    // })
  },
  watch: {
    scienceMessage(msg) {
      switch (msg.type) {
        case 'oxygen':
          this.sensor_data.oxygen = msg.percent
          // this.sensor_data.oxygen_var = msg.varianace
          break
        case 'uv':
          this.sensor_data.uv = msg.uv_index
          // this.sensor_data.uv_var = msg.varianace
          break
        case 'temperature':
          this.sensor_data.temp = msg.temperature
          // this.sensor_data.temp_var = msg.variance
          break
        case 'humidity':
          this.sensor_data.humidity = msg.relative_humidity
          // this.sensor_data.humidity_var = 100* msg.variance
          break
      }
    },
  },

  methods: {
    // randomizeSensorData() {
    //       this.sensor_data.oxygen = (19.5 + Math.random() * 4);
    //       this.sensor_data.uv = (1.5 + Math.random() * 0.5);
    //       this.sensor_data.humidity = (68 + Math.random() * 4);
    //       this.sensor_data.temp = (-2.8 + Math.random() * 2.8);
    // },
    download() {
      // downloads csv of table
      let csv = 'Oxygen, UV (index), Humidity, Temperature (C)\n'

      const numRows = this.sensor_history[0].length // transpose (flip) array
      for (let i = 0; i < numRows; ++i) {
        const row = this.sensor_history.map(sensor => sensor[i])
        csv += row.join(',') + '\n'
      }

      const anchor = document.createElement('a')
      anchor.href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv)
      anchor.download = 'sensor_data.csv'
      anchor.click()
    },
  },
}
</script>

<style scoped>
.sensor-label {
  font-size: 0.65rem;
  letter-spacing: 0.05em;
}

.min-height-0 {
  min-height: 0;
}

.min-width-0 {
  min-width: 0;
}

.flex-basis-0 {
  flex: 1 1 0 !important;
}
</style>
