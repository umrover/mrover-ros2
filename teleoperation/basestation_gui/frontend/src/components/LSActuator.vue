<template>
    <div class="wrap">
        <h3>Limit Switch Controls</h3>
        <div>
            <ToggleButton
                id="ls"
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
        lsstate: [false, false],
    }
  },

  methods:{
    ...mapActions('websocket', ['sendMessage']),
    togglels: function () {
        this.lsstate = !this.lsstate
        this.sendMessage({ type: "ls_toggle", enable: this.lsstate})
    },
  },

  created() {
    this.togglels();
  }
};
</script>

<style scoped>
.wrap {
    display: flex;
    height: 100%;
    width: 100%;
}
</style>
