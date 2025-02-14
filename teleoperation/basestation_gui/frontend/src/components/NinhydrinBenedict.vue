<template>
  <div class="wrap">
    <h3 v-if="isNinhydrin">Ninhydrin Test Controls</h3>
    <h3 v-else>Benedict's Test Controls</h3>
    <div class="box1 heaters">
      <ToggleButton
        id="heater"
        :current-state="heaters[site].enabled"
        :label-enable-text="'Heater ' + String.fromCharCode(65 + site)"
        :label-disable-text="'Heater ' + String.fromCharCode(65 + site)"
        @change="toggleHeater()"
      />
      <p :style="{ color: heaters[site].color }">
        Thermistor {{ String.fromCharCode(65 + site) }}: {{ (heaters[site].temp).toFixed(2) }} C°
      </p>
    </div>
    <div class="comms heaterStatus">
      <LEDIndicator
        :connected="heaters[site].state"
        :name="'Heater ' + String.fromCharCode(65 + site) + ' Status'"
        :show_name="true"
      />
    </div>
  </div>
</template>

<script lang="ts">
import ToggleButton from "./ToggleButton.vue";
import LEDIndicator from "./LEDIndicator.vue";
import { mapState, mapActions } from 'vuex';

export default {
  components: {
    ToggleButton,
    LEDIndicator
  },

  props: {
    site: {
      type: Number,
      required: true
    },
    isNinhydrin: { //true = Ninhydrin, false = benedict's
      type: Boolean,
      required: true
    }
  },

  data() {
    return {

      heaters: [
        {
          enabled: false,
          temp: 0,
          state: false,
          color: "grey"
        },
        {
          enabled: false,
          temp: 0,
          state: false,
          color: "grey"
        },
      ],
    };
  },

  watch: {
    message(msg) {
      if (msg.type == 'thermistors') {
        if (this.isNinhydrin) {
          this.heaters[this.site].temp = msg.temps[this.site*2+1].temperature;
        }
        else {
          this.heaters[this.site].temp = msg.temps[this.site*2].temperature;
        }
      }
      else if(msg.type == 'heater_states') {
        if (this.isNinhydrin) {
          this.heaters[this.site].state = msg.state[this.site*2+1];
        }
        else {
          this.heaters[this.site].state = msg.state[this.site*2];
        }
      }
    },
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    toggleHeater: function () {
      this.heaters[this.site].enabled = !this.heaters[this.site].enabled;
      this.sendHeaterRequest();
    },

    sendHeaterRequest: function () {
      let heaterName = String.fromCharCode(this.site+97);
      if (this.isNinhydrin) {
        heaterName += "1";
      }
      else heaterName += "0";
      this.sendMessage({id: 'ish', message: { type: "heater_enable", enabled: this.heaters[this.site].enabled, heater: heaterName}});
    },
  }
};
</script>

<style scoped>
</style>