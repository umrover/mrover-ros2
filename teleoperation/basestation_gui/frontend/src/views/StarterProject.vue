<template>
    <div class="view-wrapper">
        <h1>Body</h1>
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


    const websocketStore = useWebsocketStore()

    onMounted(() => {
        websocketStore.setupWebSocket('wypt')
        websocketStore.setupWebSocket('arm')
    })

    onUnmounted(() => {
        websocketStore.closeWebSocket('wypt')
        websocketStore.setupWebSocket('arm')
    })

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