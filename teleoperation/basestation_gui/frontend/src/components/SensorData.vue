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
            <th class='table-secondary'>Site {{ String.fromCharCode(site+65) }}</th>
            <!-- entries assumes string, any type pair -->
            <td v-for="([key, val], index) in Object.entries(sensor_data)" :key="index">{{ val.toFixed(2) }}</td>
            </tr>
        </tbody>  
        </table>

        <button class="btn btn-secondary" @click="download()">Save Data to CSV</button>
    </div>

    <div style="display: flex; flex-direction: row; gap: 10px;">
      <div style="width: 50%; overflow-x: scroll;">
        <canvas id="chart0" style="width: 100%; height: 200px; background-color: white;"></canvas>
      </div>
      <div style="width: 50%; overflow-x: scroll; margin-top: 5px;">
        <canvas id="chart1" style="width: 100%; height: 200px; background-color: white;"></canvas>
      </div>
    </div>

    <div style="display: flex; flex-direction: row; gap: 10px; margin-top: 5px;">
      <div style="width: 50%; overflow-x: scroll;">
        <canvas id="chart2" style="width: 100%; height: 200px; background-color: white;"></canvas>
      </div>
      <div style="width: 50%; overflow-x: scroll;">
        <canvas id="chart3" style="width: 100%; height: 200px; background-color: white;"></canvas>
      </div>
    </div>
</template>
  
<script lang="ts">
import { mapState } from 'vuex';
import html2canvas from "html2canvas";
import Chart from 'chart.js/auto';
  
  export default {

    props: {
        site: {
            type: Number,
            required: true
        }
    },

    mounted() {
      const charts: Chart[] = [];
      function waitForElm(selector) {
        return new Promise(resolve => {
            if (document.querySelector(selector)) {
                return resolve(document.querySelector(selector));
            }

            const observer = new MutationObserver(mutations => {
                if (document.querySelector(selector)) {
                    observer.disconnect();
                    resolve(document.querySelector(selector));
                }
            });

            // If you get "parameter 1 is not of type 'Node'" error, see https://stackoverflow.com/a/77855838/492336
            observer.observe(document.body, {
                childList: true,
                subtree: true
            });
        });
      }

      let titles = ["Oxygen Percentage Over Time (s)", "Relative Humidity Over Time (s)", "Temperature (C) Over Time (s)", "UV Index Over Time (s)"];

      // const sensor_history:number[][] = [[], [], [], []]

      let lineColors = ["#4D9DE0", "#E15554", "#3BB273", "#7768AE"]

      for (let i = 0; i < 4; ++ i){
        waitForElm(`#chart${i}`).then((el) => {
            console.log("HERE NOW.")

            // const labels = Array.from({ length: o2data.length + 1 }, (_, i) => i);
            const data = {
              labels: [],
              datasets: [{
                label: titles[i],
                data: this.sensor_history[i],
                fill: false,
                borderColor: lineColors[i],
                tension: 0.1
              }]
            };
            charts[i] = new Chart(
              document.getElementById(`chart${i}`),
                {
                  type: 'line',
                  data: data,
                  labels: [],
                  options: {
                    responsive: false,
                    scales: {
                        y: {
                            beginAtZero: true
                        }
                    }
                  }
                }
            );
        });

        
      }

      setInterval(() => {
          // console.log(Object.values(this.sensor_data).length)
          // this.$forceUpdate();
          this.sensor_history[0].push(this.sensor_data.oxygen);
          this.sensor_history[1].push(this.sensor_data.humidity);
          this.sensor_history[2].push(this.sensor_data.temp);
          this.sensor_history[3].push(this.sensor_data.uv);

          for (let x = 0; x < 4; ++x){
            if (charts[x] != null){
              charts[x].data.labels = Array.from({ length: this.sensor_history[x].length + 1 }, (_, i) => i);
              charts[x].update();
            }
          }
        }, 1000);

      
    },
  
    data() {
      return {
        sensor_data:
          {
            oxygen: 0,
            oxygen_var: 0,
            uv: 0,
            uv_var: 0,
            humidity: 0,
            humidity_var: 0,
            temp: 0,
            temp_var: 0
          },
          sensor_history: {
            oxygen: [],
            humidity: [],
            temp: [],
            uv: []
          }
      }
    },
  
    computed: {
    ...mapState('websocket', ['message']),
    sensorValues() {
      return [
        this.sensor_data.oxygen,
        this.sensor_data.oxygen_var,
        this.sensor_data.uv,
        this.sensor_data.uv_var,
        this.sensor_data.humidity,
        this.sensor_data.humidity_var,
        this.sensor_data.temp,
        this.sensor_data.temp_var
      ];
    }
  },
  created() {
      // window.setInterval();
      // this.interval = window.setInterval(() => {
      //   this.randomizeSensorData();
      // })
  },
  watch: {
    message(msg) {
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
    }
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
      let csv = "Oxygen, UV (index), Humidity, Temperature (C)\n"

      const numRows = this.sensor_history[0].length; // transpose (flip) array
      for (let i = 0; i < numRows; ++i) {
        const row = this.sensor_history.map(sensor => sensor[i]);
        csv += row.join(",") + "\n";
      }

      const anchor = document.createElement("a");
      anchor.href = "data:text/csv;charset=utf-8," + encodeURIComponent(csv);
      anchor.download = "sensor_data.csv";
      anchor.click();

        // downloads screenshot of table
      // const table = document.querySelector("#capture") as HTMLElement;
      // html2canvas(table)
      // .then(canvas => {
      //   canvas.style.display = 'none'
      //   document.body.appendChild(canvas)
      //   return canvas
      // })
      // .then(canvas => {
      //   const image = canvas.toDataURL('image/png')
      //   const a = document.createElement('a')
      //   a.setAttribute('download', 'sensor_data_site_' + String.fromCharCode(this.site+65) + '_' + new Date(Date.now()).toString() + '.png')
      //   a.setAttribute('href', image)
      //   a.click()
      //   canvas.remove()
      // })
      // alert("Downloaded report!") //remove?
    }
  }
}


</script>

<style scoped>
.sensors {
display: flex;
align-items: start;
}

</style>