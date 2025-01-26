<template>
    <!-- TODO: Add in conditonal buttons based on which buttons are clicked already -->
     <!-- we want to rotate the hex hub in one direction? -->
    <div>
        <div>
            <h2>Select Site</h2>
        </div>
        <div class="form-check form-check-inline">
            <input v-model="site" class="form-check-input" type="radio" name="flexRadioDefault" id="site0" value="0">
            <label class="form-check-label" for="site0">
                Site A
            </label>
        </div>
        <div class="form-check form-check-inline">
            <input v-model="site" class="form-check-input" type="radio" name="flexRadioDefault" id="site1" value="1">
            <label class="form-check-label" for="site1">
                Site B
            </label>
        </div>
        <div>
            <h2>Auto Shutdown Toggle</h2>
            <div class="box1 shutdown">
            <ToggleButton
                id="autoshutdown"
                :current-state="autoShutdownEnabled"
                :label-enable-text="'Auto Shutdown'"
                :label-disable-text="'Auto Shutdown'"
                @change="sendAutoShutdownCmd()"
            />
            </div>
            <div class="comms shutdownStatus">
            <LEDIndicator
                :connected="autoShutdownEnabled"
                :name="'Auto Shutdown Status'"
                :show_name="true"
            />
            </div>
        </div>
    </div>
</template>


<script lang="ts">
import { mapState, mapActions } from 'vuex';
import ToggleButton from "./ToggleButton.vue";
import LEDIndicator from "./LEDIndicator.vue";

export default {
  components: {
    ToggleButton,
    LEDIndicator
  },
    data() {
        return {
            site: 0,
            autoShutdownEnabled: true,
        };
    },

    watch: {
        site(event) {
            this.$emit("site", event);
        },
        message(msg) {
            if (msg.type == 'auto_shutoff') {
                if (!msg.success) {
                this.autoShutdownEnabled = !this.autoShutdownEnabled;
                alert('Toggling Auto Shutdown failed.')
                }
            }
        },
    },

    computed: {
    ...mapState('websocket', ['message'])
    },

    methods:{
        ...mapActions('websocket', ['sendMessage']),

        sendAutoShutdownCmd: function () {
        this.autoShutdownEnabled = !this.autoShutdownEnabled;
        this.sendMessage({ type: "auto_shutoff", shutoff: this.autoShutdownEnabled });
        },
    }
};
</script>
  
<style scoped>
</style>