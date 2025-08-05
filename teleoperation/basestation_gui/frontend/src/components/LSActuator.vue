<template>
  <div class="p-2">
    <h3 class="m-0 p-0">Limit Switch</h3>
    <div class="m-0 p-0">
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

<script lang="ts">
import Vuex from 'vuex'
const { mapActions } = Vuex
import ToggleButton from './ToggleButton.vue'
export default {
  components: {
    ToggleButton,
  },
  data() {
    return {
      lsstate: [false, false],
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
    togglels: function () {
      this.lsstate = !this.lsstate
      this.$store.dispatch('websocket/sendMessage', {
        id: 'science',
        message: {
          type: 'ls_toggle',
          enable: this.lsstate,
        },
      })
    },
  },

  created() {
    this.togglels()
  },
}
</script>