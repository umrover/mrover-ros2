<template>
  <div class="flex flex-col gap-2 h-full">
    <div class="flex justify-between items-center">
      <h4 class="component-header">Arm Controls</h4>
      <p 
      class="text-danger"
      :class="forcing_limit === true ? 'visible' : 'invisible'">
        Limit Reached!
      </p>
      <IndicatorDot :is-active="connected" class="mr-2" />
    </div>
    <div class="btn-group w-full" role="group" aria-label="Arm mode selection" data-testid="pw-arm-mode-buttons">
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="mode === 'disabled' ? 'cmd-btn-danger' : 'cmd-btn-outline-danger'"
          data-testid="pw-arm-mode-disabled"
          @click="newRAMode('disabled')"
        >
          Disabled
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="mode === 'throttle' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-throttle"
          @click="newRAMode('throttle')"
        >
          Throttle
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="mode === 'ik-pos' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-ik-pos"
          @click="newRAMode('ik-pos')"
        >
          IK Pos
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="mode === 'ik-vel' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-ik-vel"
          @click="newRAMode('ik-vel')"
        >
          IK Vel
        </button>
      </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" layout="horizontal" class="grow min-h-0" />
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'
import { armAPI } from '@/utils/api'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage, ThrottleMessage/*, VelocityMessage*/ } from '@/types/websocket'




const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const mode = ref('disabled')
const forcing_limit = ref(false)
const limits_hit_external = ref([0, 0, 0, 0, 0, 0])

const { connected, axes, buttons, vibrationActuator } = useGamepadPolling({
  controllerIdFilter: 'Microsoft',
  topic: 'arm',
  messageType: 'ra_controller',
})

const keyDown = async (event: { key: string }) => {
  if (event.key === ' ') {
    await newRAMode('disabled')
  }
}

onMounted(() => {
  document.addEventListener('keydown', keyDown)
})

onBeforeUnmount(() => {
  document.removeEventListener('keydown', keyDown)
})

const armMessage = computed(() => messages.value['arm'])

const newRAMode = async (newMode: string) => {
  try {
    mode.value = newMode
    const data = await armAPI.setRAMode(mode.value)
    if (data.status === 'success' && data.mode) {
      mode.value = data.mode
    }
  } catch (error) {
    console.error('Failed to set arm mode:', error)
  }
}

// Defined outside as not to recalculate every time
let jointThrIdx = []

watch(armMessage, (msg: unknown) => {
if (!msg || typeof msg !== 'object') return

  // const gamepads = navigator.getGamepads()
  // const gamepad = gamepads.find(gp => gp && gp.id.includes('Microsoft'))
  // console.log(gamepad)

  if ('type' in msg && msg.type === "arm_state"){
    const typedMsg = msg as ControllerStateMessage
    for(let i = 0; i < 6; ++i){
      limits_hit_external.value[i] = typedMsg.limits_hit[i]
    }
  }

  if ('type' in msg && msg.type === "arm_throttle_command"){
    const typedMessage = msg as ThrottleMessage
    forcing_limit.value = false;

    // Find what index is what joint
    // console.log(typedMessage.names.length)
    if(jointThrIdx.length == 0){
      for(let i = 0; i < typedMessage.names.length; ++i){
        let currName = typedMessage.names[i]
        switch(currName){
          case "joint_a":
            jointThrIdx.push(0);
            break;
          case "joint_b":
            jointThrIdx.push(1);
            break;
          case "joint_c":
            jointThrIdx.push(2);
            break;
          case "joint_de_pitch":
            jointThrIdx.push(3);
            break;
          case "joint_de_roll":
            jointThrIdx.push(4);
            break;
          case "gripper":
            jointThrIdx.push(5);
            break;
          case "cam":
            jointThrIdx.push(6);
            break;
          default:
        }
      }
      console.log("Joint thrust indexes:")
      console.log(jointThrIdx)
    }

    // Determine if limits hit
    for(let i = 0; i < jointThrIdx.length; ++i){
      if(limits_hit_external.value[jointThrIdx[i]] == 1 && typedMessage.throttles[i] < 0){
        forcing_limit.value = true
      } else if (limits_hit_external.value[jointThrIdx[i]] == 2 && typedMessage.throttles[i] > 0) {
        forcing_limit.value = true
      }
    }

    // // Joint A
    // if(limits_hit_external.value[0] == 1 && typedMessage.throttles[0] < 0){
    //   forcing_limit.value = true
    // } else if (limits_hit_external.value[0] == 2 && typedMessage.throttles[0] > 0) {
    //   forcing_limit.value = true
    // }

    // // Joint B
    // if(limits_hit_external.value[1] == 1 && typedMessage.throttles[5] < 0){
    //   forcing_limit.value = true
    // } else if (limits_hit_external.value[1] == 2 && typedMessage.throttles[5] > 0) {
    //   forcing_limit.value = true
    // }

    // // Joint C
    // if(limits_hit_external.value[2] == 1 && typedMessage.throttles[6] < 0){
    //   forcing_limit.value = true
    // } else if (limits_hit_external.value[2] == 2 && typedMessage.throttles[6] > 0) {
    //   forcing_limit.value = true
    // }

    // // Roll Joint
    // if(limits_hit_external.value[4] == 1 && typedMessage.throttles[6] < 0){
    //   forcing_limit.value = true
    // } else if (limits_hit_external.value[4] == 2 && typedMessage.throttles[6] > 0) {
    //   forcing_limit.value = true
    // }
  }

  if(forcing_limit.value){
    vibrationActuator.value.playEffect('dual-rumble', {
      startDelay: 0,
      duration: 100,
      weakMagnitude: 0.1,
      strongMagnitude: 0,})
  }

  // if ('type' in msg && msg.type === "arm_velocity_command"){
  //   console.log("read")

  //   // const typedMessage = msg as VelocityMessage
  //   // console.log(typedMessage)
  // }
})
</script>

