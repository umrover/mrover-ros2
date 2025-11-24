<template>
  <div class="sp-controls d-flex flex-column align-items-center">
    <h4 class="m-0 font-monospace">SP Arm</h4>
    <div class="gamepad-container">
      <div
        id="gamepad"
        :class="{ disconnected: !controllerConnected }"
        data-color="black"
      >
        <template v-if="controllerConnected">
          <div class="triggers">
            <span
              class="trigger left"
              :data-value="buttons[6]"
              :style="triggerStyle(buttons[6])"
            ></span>
            <span
              class="trigger right"
              :data-value="buttons[7]"
              :style="triggerStyle(buttons[7])"
            ></span>
          </div>
          <div class="bumpers">
            <span class="bumper left" :data-pressed="buttonPressed(4)"></span>
            <span class="bumper right" :data-pressed="buttonPressed(5)"></span>
          </div>
          <div class="arrows">
            <span class="select" :data-pressed="buttonPressed(8)"></span>
            <span class="start" :data-pressed="buttonPressed(9)"></span>
          </div>
          <div class="buttons">
            <span class="button a" :data-pressed="buttonPressed(0)"></span>
            <span class="button b" :data-pressed="buttonPressed(1)"></span>
            <span class="button x" :data-pressed="buttonPressed(2)"></span>
            <span class="button y" :data-pressed="buttonPressed(3)"></span>
          </div>
          <div class="sticks">
            <span
              class="stick left"
              :data-pressed="buttonPressed(10)"
              :style="stickStyle(axes[0], axes[1])"
            ></span>
            <span
              class="stick right"
              :data-pressed="buttonPressed(11)"
              :style="stickStyle(axes[2], axes[3])"
            ></span>
          </div>
          <div class="dpad">
            <span class="face up" :data-pressed="buttonPressed(12)"></span>
            <span class="face down" :data-pressed="buttonPressed(13)"></span>
            <span class="face left" :data-pressed="buttonPressed(14)"></span>
            <span class="face right" :data-pressed="buttonPressed(15)"></span>
          </div>
        </template>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

const UPDATE_HZ = 20

const websocketStore = useWebsocketStore()

const gamepadConnected = ref(false)
const controllerConnected = computed(() => gamepadConnected.value)

const axes = ref<number[]>([0, 0, 0, 0])
const buttons = ref<number[]>(new Array(17).fill(0))

let interval: number | undefined

function buttonPressed(index: number): string {
  return buttons.value[index] > 0.5 ? 'true' : 'false'
}

function triggerStyle(value: number) {
  return {
    opacity: value === 0 ? 0 : 1,
    clipPath: `inset(${(1 - value) * 100}% 0px 0px 0px)`,
  }
}

function stickStyle(axisX: number, axisY: number) {
  return {
    marginTop: `${axisY * 25}px`,
    marginLeft: `${axisX * 25}px`,
    transform: `rotateX(${-axisY * 30}deg) rotateY(${axisX * 30}deg)`,
  }
}

onMounted(() => {
  interval = window.setInterval(() => {
    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(
      gamepad => gamepad && gamepad.id.includes('Microsoft'),
    )
    gamepadConnected.value = !!gamepad
    if (!gamepad) return

    axes.value = [...gamepad.axes]
    buttons.value = gamepad.buttons.map(button => button.value)

    websocketStore.sendMessage('science', {
      type: 'sp_controller',
      axes: gamepad.axes,
      buttons: gamepad.buttons.map(button => button.value),
    })
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  if (interval !== undefined) {
    window.clearInterval(interval)
  }
})
</script>

<style scoped>
.sp-controls {
  padding: 8px;
}


.gamepad-container {
  width: 150px;
  height: 126px;
  overflow: hidden;
}

#gamepad {
  height: 315px;
  width: 375px;
  position: relative;
  background-size: contain;
  background-repeat: no-repeat;
  background-position: center;
  transform: scale(0.4);
  transform-origin: top left;
}

#gamepad[data-color='black'] {
  background-image: url('/gamepad/xbox/base-black.svg');
}

#gamepad[data-color='white'] {
  background-image: url('/gamepad/xbox/base-white.svg');
}

#gamepad.disconnected {
  background-image: url('/gamepad/xbox/disconnected.svg');
}

#gamepad.disconnected div {
  display: none;
}

#gamepad .triggers {
  width: 224px;
  height: 61px;
  position: absolute;
  left: 75.5px;
}

#gamepad .trigger {
  width: 44.5px;
  height: 61px;
  background: url('/gamepad/xbox/trigger.svg');
  background-size: contain;
  clip-path: inset(100% 0px 0px 0px);
}

#gamepad .trigger[data-value='0'] {
  opacity: 0;
}

#gamepad .trigger.left {
  float: left;
  background-position: 0 0;
}

#gamepad .trigger.right {
  float: right;
  transform: rotateY(180deg);
}

#gamepad .bumper {
  width: 85px;
  height: 30.5px;
  background: url('/gamepad/xbox/bumper.svg');
  background-size: contain;
  opacity: 0;
}

#gamepad .bumpers {
  position: absolute;
  width: 268px;
  height: 30.5px;
  left: 53.5px;
  top: 64.5px;
}

#gamepad .bumper[data-pressed='true'] {
  opacity: 1;
}

#gamepad .bumper.left {
  float: left;
}

#gamepad .bumper.right {
  float: right;
  transform: rotateY(180deg);
}

#gamepad .arrows {
  position: absolute;
  width: 70.5px;
  height: 16.5px;
  top: 132px;
  left: 153px;
}

#gamepad .select,
#gamepad .start {
  background: url('/gamepad/xbox/start-select.svg');
  background-size: 66px 16.5px;
  width: 16.5px;
  height: 16.5px;
  opacity: 0;
}

#gamepad .select[data-pressed='true'],
#gamepad .start[data-pressed='true'] {
  opacity: 1;
}

#gamepad .select {
  float: left;
}

#gamepad .start {
  background-position: 16.5px 0px;
  float: right;
}

#gamepad .buttons {
  position: absolute;
  width: 77.5px;
  height: 78px;
  top: 100.5px;
  left: 244.5px;
}

#gamepad .button {
  position: absolute;
  background: url('/gamepad/xbox/buttons.svg');
  background-size: 106px 53px;
  width: 26.5px;
  height: 26.5px;
}

#gamepad .button[data-pressed='true'] {
  background-position-y: -26.5px;
  opacity: 1;
}

#gamepad .a {
  background-position: 0 0;
  top: 51px;
  left: 25.5px;
}

#gamepad .b {
  background-position: -26.5px 0;
  top: 26px;
  right: 0.5px;
}

#gamepad .x {
  background-position: -53px 0;
  top: 26px;
  left: 0.5px;
}

#gamepad .y {
  background-position: -79.5px 0;
  top: 0.5px;
  left: 25.5px;
}

#gamepad .sticks {
  position: absolute;
  width: 185.5px;
  height: 98px;
  top: 119.5px;
  left: 72px;
}

#gamepad .stick {
  position: absolute;
  background: url('/gamepad/xbox/stick.svg');
  background-size: 84px 41.5px;
  background-position: -42.5px 0;
  height: 41.5px;
  width: 41.5px;
}

#gamepad .stick[data-pressed='true'] {
  background-position: 0 0;
}

#gamepad .stick.left {
  top: 0;
  left: 0;
}

#gamepad .stick.right {
  top: 56.5px;
  left: 144px;
}

#gamepad .dpad {
  position: absolute;
  width: 55px;
  height: 55.5px;
  top: 172.5px;
  left: 111.5px;
}

#gamepad .face {
  background: url('/gamepad/xbox/dpad.svg');
  background-size: 35px 64px;
  position: absolute;
  opacity: 0;
}

#gamepad .face[data-pressed='true'] {
  opacity: 1;
}

#gamepad .face.up {
  background-position: 17.5px 0;
  left: 19px;
  top: 0.5px;
  width: 17px;
  height: 28px;
}

#gamepad .face.down {
  left: 19px;
  bottom: 0;
  width: 17px;
  height: 28px;
}

#gamepad .face.left {
  background-position: 0 -46.5px;
  width: 28px;
  height: 17px;
  top: 19.5px;
  left: 0;
}

#gamepad .face.right {
  background-position: 0 -28.5px;
  width: 28px;
  height: 17px;
  top: 19.5px;
  right: 0;
}

.clear {
  clear: both;
}
</style>
