<template>
  <div class='wrap box'>
    <h3>Soil Data</h3>
    <div class='table-responsive'>
      <table class='table' id="capture">
        <thead>
        <tr class='table-primary'>
          <th scope='col'>Temperature</th>
          <th scope='col'>Humidity</th>
        </tr>
        </thead>
        <tbody>
        <tr class='bold-border'>
          <td>{{ temp.toFixed(2) }}ºC</td>
          <td>{{ (humidity * 100).toFixed(2) }}%</td>
        </tr>
        </tbody>
      </table>
      <!-- <div>
        <p v-if="predictedTemp"> Predicted Temperature: {{ predictedTemp }}</p>
        <Checkbox :name="'Read Temp Data'" @toggle="readData = $event"></Checkbox>
      </div> -->
      <button class="btn btn-secondary" @click="download()">Save Data to CSV</button>
    </div>
  </div>
</template>

<script lang='ts'>

import { mapState, mapActions } from 'vuex'
import Checkbox from './Checkbox.vue'
import html2canvas from 'html2canvas';

export default {
  components: {
    Checkbox
  },

  data() {
    return {
      temp: 0,
      humidity: 0,
      tempArray: [] as number[],
      timestamps: [] as number[],
      readData: false,
      prevState: false,
      exponents: [],
      predictedTemp: null
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  watch: {
    message(msg) {
      switch (msg.type) {
        case 'soil_temp':
          this.temp = msg.temperature
          if(this.readData){
            this.tempArray.push(this.temp)
            this.timestamps.push(Date.now())
          }
          else {
            this.predictedTemp = this.predictTemp(Date.now())
          }
          break
        case 'soil_humidity':
          this.humidity = msg.relative_humidity
          break
        case 'poly_fit':
          this.exponents = msg.exponents
          break
      }
    },

    readData() {
      if(!this.readData){
        this.publishPolyfit()
      }
      else if(this.readData && this.readData != this.prevState) {
        this.exponents = []
        this.tempArray = []
        this.timestamps = []
      }
      this.prevState = this.readData
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    publishPolyfit: function(){
      this.sendMessage('sa', {type: "poly_fit", temperatures: this.tempArray, timestamps: this.timestamps})
    },

    predictTemp: function(timestamp: any) {
      const val = this.exponents[0] * timestamp + this.exponents[1]
      return Math.exp(val)
    },


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
    }
  }
}
</script>

<style scoped></style>
  