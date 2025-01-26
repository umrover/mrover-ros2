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
        @change="toggleHeater(site)"
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

let interval;

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
        {
          enabled: false,
          temp: 0,
          state: false,
          color: "grey"
        }
      ],

      autoShutdownEnabled: true,
    };
  },

  watch: {
    message(msg) {
      if (msg.type == 'thermistor') {
        if(this.site == 0) return;
        if (this.isNinhydrin) {
          this.heaters[this.site].temp = msg.temps[this.site*2+1].temperature;
        }
        else {
          this.heaters[this.site].temp = msg.temps[this.site*2].temperature;
        }
      }
      else if(msg.type == 'heater_states') {
        if(this.site == 0) return;
        if (this.isNinhydrin) {
          this.heaters[this.site].state = msg.state[this.site*2+1];
        }
        else {
          this.heaters[this.site].state = msg.state[this.site*2];
        }
      }
    },
  },

  beforeUnmount: function () {
    window.clearInterval(interval);
  },

  created: function () {
    interval = window.setInterval(() => {
      this.sendHeaterRequest(this.site);
    }, 100);    
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    toggleHeater: function (id) {
      this.heaters[id].enabled = !this.heaters[id].enabled;
      this.sendHeaterRequest(id);
    },

    sendHeaterRequest: function (id) {
      let heaterName = "b";
      if (this.isNinhydrin) {
        heaterName = "n";
      }
      heaterName += id;
      this.sendMessage({ type: "heater_enable", enabled: this.heaters[id].enabled, heater: heaterName});
    },

    capturePhoto() {
      this.sendMessage({ type: "capture_photo" });
    }
  }
};
</script>

<style scoped>
</style>