<template>
    <div class="view-wrapper">
        <div class="d-flex flex-col gap-2 mb-2">
            <button class="btn btn-primary" @click="spamTestMessages()">
                Spam test messages
            </button>
            <ArmControls class="island py-2" />
            <Rover3D class="island m-0 p-0" style="max-height: 700px;" />
        </div>
    </div>
</template>

<script lang="ts" setup>
    import { onMounted, onUnmounted } from 'vue'
    import { useWebsocketStore } from '@/stores/websocket'
    import ArmControls from '../components/ArmControls.vue'
    import Rover3D from '../components/Rover3D.vue'

    // You may also see code in other files that does:
    // --- const webSocketStore = useWebsocketStore() ---
    // and calls methods using it, such as
    // --- webSocketStore.sendMessage(arguments)
    const {setupWebSocket, closeWebSocket, sendMessage} = useWebsocketStore()

    // Set up websockets when this component is mounted
    // (happens shortly after page load here)
    onMounted(() => {
        setupWebSocket('wypt')
        setupWebSocket('arm')
    })

    // Close websockets when this component is unmounted
    // (happens during page unload, in this case)
    onUnmounted(() => {
        closeWebSocket('wypt')
        closeWebSocket('arm')
    })

    // Send a bunch of messages to the "wypt" websocket
    const spamTestMessages = () => {

        // Send a message every 1000 miliseconds
        const interval = setInterval(() => {
            sendMessage('wypt', 
            {
                type: 'debug',
                timestamp: new Date().toISOString(),
            })
        }, 1000)

        // Stop sending messages after 5000 miliseconds
        setTimeout(() => clearInterval(interval), 5000)
    }
</script>