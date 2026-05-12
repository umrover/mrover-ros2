<template>
    <div class="view-wrapper">
        <h1>Hello World!</h1>
        <!-- TODO add button -->
        <!--TODO add components-->
    </div>
</template>

<script lang="ts" setup>
    import { onMounted, onUnmounted } from 'vue'
    import { useWebsocketStore } from '@/stores/websocket'
    import { StringMessage } from '@/types/websocket'
    // TODO import components

    // You may also see code in other files that does:
    // --- const webSocketStore = useWebsocketStore() ---
    // and calls methods using it, such as
    // --- webSocketStore.sendMessage(arguments)
    const { setupWebSocket, closeWebSocket, sendMessage, onMessage } = useWebsocketStore()

    // Set up websockets when this component is mounted
    // (happens shortly after page load here)
    onMounted(() => {
        setupWebSocket('starter')
        // TODO add necessary sockets
    })

    // Close websockets when this component is unmounted
    // (happens during page unload, in this case)
    onUnmounted(() => {
        closeWebSocket('starter')
        // TODO add necessary sockets
    })

    // Send a bunch of messages to the "starter" websocket
    const spamTestMessages = () => {

        // Send a message every 1000 milliseconds
        const interval = setInterval(() => {
            sendMessage('starter', {
                type: 'debug',
                timestamp: new Date().toISOString(),
            })
        }, 1000)

        // Stop sending messages after 5000 miliseconds
        setTimeout(() => clearInterval(interval), 5000)
    }

    // TODO add onMessage
</script>