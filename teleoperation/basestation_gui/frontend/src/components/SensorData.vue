<template>
    <h3>Sensor Data</h3>
    <div class="sensors align-items-center">
        <table class="table table-bordered mx-3 mb-0" id="capture">
        <thead>
            <tr class="table-primary">
            <th></th>
            <th colspan="2">Oxygen (%)</th>
            <th colspan="2">Methane (ppm)</th>
            <th colspan="2">UV (index)</th>
            <th colspan="2">Humidity (%)</th>
            <th colspan="2">Temp (°C)</th>
            </tr>
        </thead>
        <tbody>
            <tr>
            <th class='table-secondary'>Site {{ String.fromCharCode(site+65) }}</th>
            <td v-for="val in Object.values(sensor_data)" :key="val">{{ val.toFixed(2) }}</td>
            </tr>
        </tbody>  
        </table>

        <button class="btn btn-secondary" @click="download()">Save Data to CSV</button>
    </div>
</template>
  
<script lang="ts">
import { mapState } from 'vuex';
import html2canvas from "html2canvas";
  
  export default {

    props: {
        site: {
            type: Number,
            required: true
        }
    },
  
    data() {
      return {
        sensor_data:
          {
            oxygen: 0,
            oxygen_var: 0,
            methane: 0,
            methane_var: 0,
            uv: 0,
            uv_var: 0,
            humidity: 0,
            humidity_var: 0,
            temp: 0,
            temp_var: 0
          }
      }
    },
  
    computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      switch (msg.type) {
        case 'oxygen':
            this.sensor_data.oxygen = msg.percent
            this.sensor_data.oxygen_var = msg.varianace
            break
        case 'methane':
            this.sensor_data.methane_var = msg.ppm
            this.sensor_data.methane_var = msg.varianace
            break
        case 'uv':
            this.sensor_data.uv = msg.uv_index
            this.sensor_data.uv_var = msg.varianace
            break
        case 'temperature':
            this.sensor_data.temp = msg.temperature
            this.sensor_data.temp_var = msg.variance
            break
        case 'humidity':
            this.sensor_data.humidity = 100 * msg.relative_humidity
            this.sensor_data.humidity_var = 100* msg.variance
            break
      }
    }
  },

  methods: {
    download() {
        // downloads screenshot of table
      const table = document.querySelector("#capture") as HTMLElement;
      html2canvas(table)
      .then(canvas => {
        canvas.style.display = 'none'
        document.body.appendChild(canvas)
        return canvas
      })
      .then(canvas => {
        const image = canvas.toDataURL('image/png')
        const a = document.createElement('a')
        a.setAttribute('download', 'sensor_data_site_' + String.fromCharCode(this.site+65) + '_' + new Date(Date.now()).toString() + '.png')
        a.setAttribute('href', image)
        a.click()
        canvas.remove()
      })
      alert("Downloaded report!") //remove?
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