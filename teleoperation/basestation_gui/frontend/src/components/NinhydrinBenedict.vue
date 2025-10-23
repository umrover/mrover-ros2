<template>
  <div class="wrap">
    <h3 v-if="isNinhydrin">Ninhydrin Test Controls</h3>
    <h3 v-else>Benedict's Test Controls</h3>
    <div class="box1 heaters">
      <ToggleButton
        id="heater"
        :current-state="heaters[0].enabled"
        :label-enable-text="'Heater A'"
        :label-disable-text="'Heater A'"
        @change="toggleHeater(0)"
      />
      <p :style="{ color: heaters[0].color }">
        Thermistor {{ 'A' }}:
        {{ heaters[0].temp.toFixed(2) }} C°
      </p>
    </div>
    <div class="comms heaterStatus">
      <LEDIndicator
        :connected="heaters[0].state"
        :name="'Heater A Status'"
        :show_name="true"
      />
    </div>

    <div class="box1 heaters">
      <ToggleButton
        id="heater"
        :current-state="heaters[1].enabled"
        :label-enable-text="'Heater B'"
        :label-disable-text="'Heater B'"
        @change="toggleHeater(1)"
      />
      <p :style="{ color: heaters[1].color }">
        Thermistor {{ 'B' }}:
        {{ heaters[1].temp.toFixed(2) }} C°
      </p>
    </div>
    <div class="comms heaterStatus">
      <LEDIndicator
        :connected="heaters[1].state"
        :name="'Heater B Status'"
        :show_name="true"
      />
    </div>
  </div>
</template>

<script lang="ts">
import ToggleButton from './ToggleButton.vue'
import LEDIndicator from './LEDIndicator.vue'
import Vuex from 'vuex'
import type { WebSocketState } from '../types/websocket'
const { mapState, mapActions } = Vuex

export default {
  components: {
    ToggleButton,
    LEDIndicator,
  },

  props: {
    site: {
      type: Number,
      required: true,
    },
    isNinhydrin: {
      //true = Ninhydrin, false = benedict's
      type: Boolean,
      required: true,
    },
  },

  data() {
    return {
      heaters: [
        {
          enabled: false,
          temp: 0,
          state: false,
          color: 'grey',
        },
        {
          enabled: false,
          temp: 0,
          state: false,
          color: 'grey',
        },
      ],
    }
  },

  computed: {
    ...mapState('websocket', {
      scienceMessage: (state: WebSocketState) => state.messages['science']
    }),
  },

  watch: {
    scienceMessage(msg) {
      if (msg.type == 'thermistors') {
        if (this.isNinhydrin) {
          this.heaters[0].temp =
            msg.temps[0 * 2 + 1].temperature

          this.heaters[1].temp =
            msg.temps[1 * 2 + 1].temperature
        } else {
          this.heaters[0].temp = msg.temps[0 * 2].temperature
          
          this.heaters[1].temp = msg.temps[1 * 2].temperature
        }
      } else if (msg.type == 'heater_states') {
        if (this.isNinhydrin) {
          this.heaters[0].state = msg.state[0 * 2 + 1]
          this.heaters[0].enabled = this.heaters[0].state

          this.heaters[1].state = msg.state[1 * 2 + 1]
          this.heaters[1].enabled = this.heaters[1].state
        } else {
          this.heaters[0].state = msg.state[0 * 2]
          this.heaters[0].enabled = this.heaters[0].state

          this.heaters[1].state = msg.state[1 * 2]
          this.heaters[1].enabled = this.heaters[1].state
        }
      }
    },
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    toggleHeater: function (site_num: number) {
      this.heaters[site_num].enabled = !this.heaters[site_num].enabled
      this.sendHeaterRequest(site_num)
    },

    sendHeaterRequest: function (site_num: number) {
      let heaterName = String.fromCharCode(site_num + 97)
      if (this.isNinhydrin) {
        heaterName += '1'
      } else heaterName += '0'
      this.$store.dispatch('websocket/sendMessage', {
        id: 'science',
        message: {
          type: 'heater_enable',
          enable: this.heaters[site_num].enabled,
          heater: heaterName,
        },
      })
    },
  },
}
</script>

<style scoped></style>
