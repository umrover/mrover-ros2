<template>
  <div class='wrap'>
    <div v-show='false' id='key'>
      <input @keydown='keyMonitorDown' />
      <input @keyup='keyMonitorUp' />
    </div>
  </div>
</template>

<script lang='ts' setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

const websocketStore = useWebsocketStore()

const mappings = {
  w: 0,
  a: 1,
  s: 2,
  d: 3
}
const keys = ref(Array(4).fill(0))

let interval: number | undefined = undefined

const UPDATE_HZ = 20

const keyMonitorDown = (event: { key: string }) => {
  const index = mappings[event.key.toLowerCase()]
  if (index === undefined) return

  keys.value[index] = 1
}

const keyMonitorUp = (event: { key: string }) => {
  const index = mappings[event.key.toLowerCase()]
  if (index === undefined) return

  keys.value[index] = 0
}

const publish = () => {
  websocketStore.sendMessage('mast', {
    type: 'mast_keyboard',
    axes: [],
    buttons: keys.value
  })
}

onMounted(() => {
  document.addEventListener('keydown', keyMonitorDown)
  document.addEventListener('keyup', keyMonitorUp)
  interval = window.setInterval(() => {
    publish()
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  window.clearInterval(interval)
  document.removeEventListener('keyup', keyMonitorUp)
  document.removeEventListener('keydown', keyMonitorDown)
})
</script>

<style scoped>
.wrap {
  display: none;
}
</style>
