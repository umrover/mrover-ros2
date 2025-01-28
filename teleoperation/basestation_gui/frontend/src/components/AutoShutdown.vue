<template>
    <div>
        <h2>Heater Auto Shutdown</h2>
        <ToggleButton
            id="autoshutdown"
            :current-state="autoShutdownEnabled"
            :label-enable-text="'Auto Shutdown'"
            :label-disable-text="'Auto Shutdown'"
            @change="sendAutoShutdownCmd()"
        />
        <LEDIndicator
            :connected="autoShutdownEnabled"
            :name="'Auto Shutdown Status'"
            :show_name="true"
        />
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
            autoShutdownEnabled: false,
        };
    },

    watch: {
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
        console.log("sending!")
        },
    },

    created() {
        this.sendAutoShutdownCmd();
    },
};
</script>
  
<style scoped>
</style>