<template>
    <div class="wrap">
        <h3>Pump and Limit Switch Controls</h3>
        <div>
            <ToggleButton
                id="pump0"
                :current-state="p0state"
                :label-enable-text="'Pump 0'"
                :label-disable-text="'Pump 0'"
                @change="togglep0()"
            />
            <ToggleButton
                id="pump1"
                :current-state="p1state"
                :label-enable-text="'Pump 1'"
                :label-disable-text="'Pump 1'"
                @change="togglep1()"
            />
            <ToggleButton
                id="pump0"
                :current-state="lsstate"
                :label-enable-text="'Limit Switch Sensor Actuator'"
                :label-disable-text="'Limit Switch Sensor Actuator'"
                @change="togglels()"
            />
        </div>
    </div>
</template>

<script lang='ts'>
import { mapState, mapActions } from 'vuex';
import ToggleButton from './ToggleButton.vue';
export default {
  components: {
    ToggleButton
  },
  data() {
    return {
        p0state: false, 
        p1state: false,
        lsstate: false,
    }
  },

  methods:{
    ...mapActions('websocket', ['sendMessage']),

    togglep0: function () {
        this.p0state = !this.p0state
        console.log(this.p0state)
        this.sendMessage({ type: "p0_toggle", enable: this.p0state})
    },
    togglep1: function () {
        this.p1state = !this.p1state
        this.sendMessage({ type: "p1_toggle", enable: this.p1state})
    },
    togglels: function () {
        this.lsstate = !this.lsstate
        this.sendMessage({ type: "ls_toggle", enable: this.lsstate})
    },
  },

  created() {
    this.togglep0();
    this.togglep1();
    this.togglels();
  }
};
</script>

<style scoped>
.wrap {
    display: flex;
    /* align-items: center; */
    height: 100%;
    width: 100%;
}
</style>
