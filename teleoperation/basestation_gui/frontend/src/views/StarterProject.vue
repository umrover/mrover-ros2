<template>
    <div class="view-wrapper">
        <h1>Body</h1>
        <button class="btn btn-primary" @click="spamTestMessages()">
            Spam test messages
        </button>
    </div>
</template>

<script lang="ts" setup>
    import { onMounted, onUnmounted } from 'vue'
    // import type { WebSocketState } from '../types/websocket'
    import { useWebsocketStore } from '@/stores/websocket'


    const websocketStore = useWebsocketStore()

    onMounted(() => {
        
    })

    onUnmounted(() => {
        
    })

    websocketStore.setupWebSocket('wypt')

    

    const spamTestMessages = () => {
        websocketStore.sendMessage('wypt', 
        {
            type: 'debug',
            timestamp: new Date().toISOString(),
        })
    }

    // this.store.dispatch('websocket/sendMessage', {
    //     id: 'waypoint',
    //     message: {
    //         type: 'debug',
    //         timestamp: new Date().toISOString(),
    //     }
    // })

    // export default defineComponent({
    //     components: {
    //             // TODO
    //         },

    //     mounted() {
    //         this.$store.dispatch('websocket/setupWebSocket', 'waypoints')
    //             // TODO
    //     },

    //     unmounted() {
    //         this.$store.dispatch('websocket/closeWebSocket', 'waypoints')
    //             // TODO
    //     },

    //     computed: {
    //         ...mapState('websocket', {
    //         waypointsMessage: (state: WebSocketState) => state.messages['waypoints'],
    //         }),
    //     },

    //     watch: {
    //         waypointsMessage(msg) {
    //         console.log(msg)
    //         },
    //     },

    //     methods: {
    //         spamTestMessages() {
    //         const interval = setInterval(() => {
    //             this.$store.dispatch('websocket/sendMessage', {
    //             id: 'waypoints',
    //             message: {
    //                 type: 'debug',
    //                 timestamp: new Date().toISOString(),
    //             },
    //             })
    //         }, 1000)
    //         setTimeout(() => clearInterval(interval), 5000)
    //         },
    //     },
    // })
</script>