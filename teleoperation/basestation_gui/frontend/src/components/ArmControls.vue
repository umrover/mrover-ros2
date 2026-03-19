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
    <div class="flex w-full" role="group" aria-label="Arm mode selection" data-testid="pw-arm-mode-buttons">
      <div class="cmd-btn-group-connected w-full">
        <button
          type="button"
          class="cmd-btn cmd-btn-sm flex-1"
          :class="mode === 'disabled' ? 'cmd-btn-danger' : 'cmd-btn-outline-danger'"
          data-testid="pw-arm-mode-disabled"
          @click="newRAMode('disabled')"
          >
          Disabled
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm flex-1"
          :class="isStowing ? 'cmd-btn-warning' : 'cmd-btn-outline-warning'"
          :disabled="isStowing"
          @click="stowArm"
        >
          Stow
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm flex-1"
          :class="mode === 'throttle' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-throttle"
          @click="newRAMode('throttle')"
        >
          Throttle
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm flex-1"
          :class="mode === 'ik-pos' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-ik-pos"
          @click="newRAMode('ik-pos')"
        >
          IK Pos
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm flex-1"
          :class="mode === 'ik-vel' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-ik-vel"
          @click="newRAMode('ik-vel')"
        >
          IK Vel
        </button>
      </div>
    </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" layout="horizontal" class="grow min-h-0" />
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { armAPI } from '@/utils/api'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import { useWebsocketStore } from '@/stores/websocket'
import type { IkFeedbackMessage, ControllerStateMessage, ThrottleMessage } from '@/types/websocket'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'

const { onMessage } = useWebsocketStore()

const mode = ref('disabled')
const forcing_limit = ref(false)

const isStowing = ref(false)
const stowTarget = ref<{ x: number; y: number; z: number } | null>(null)

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

onMessage<IkFeedbackMessage>('arm', 'ik_feedback', (msg) => {
  console.log('yippee:', msg.pos)
})

const stowArm = async () => {
  try {
    isStowing.value = true
    const result = await armAPI.stowArm()
    if (result.status === 'success') {
      mode.value = 'stow'
      stowTarget.value = {
        x: result.stow_target.pos.x,
        y: result.stow_target.pos.y,
        z: result.stow_target.pos.z,
      }
    } else {
      isStowing.value = false
    }
  } catch (error) {
    console.error('Failed to start stow:', error)
    isStowing.value = false
  }
}

const newRAMode = async (newMode: string) => {
  try {
    isStowing.value = false
    mode.value = newMode
    const data = await armAPI.setRAMode(mode.value)
    if (data.status === 'success' && data.mode) {
      mode.value = data.mode
    }
  } catch (error) {
    console.error('Failed to set arm mode:', error)
  }
}

// Indexes coressponding to joints in this order:
// joint_a, joint_b, joint_c, joint_de_pitch, joint_de_roll, gripper, cam
let jointThrIdx = []

let limits_hit_external = ref<number[]>([0, 0, 0, 0, 0, 0])

onMessage<ControllerStateMessage>('arm', 'arm_state', (msg) => {
  for(let i = 0; i < 6; ++i){
    limits_hit_external.value[i] = msg.limits_hit[i]
  }
})

onMessage<ThrottleMessage>('arm', 'arm_throttle_command', (msg) => {
  forcing_limit.value = false;

  // Find what index is what joint. Only done first time thrust input is recieved
  if(jointThrIdx.length == 0){
    for(let i = 0; i < msg.names.length; ++i){
      let currName = msg.names[i]
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
  }

  // Determine if limits hit
  for(let i = 0; i < jointThrIdx.length; ++i){
    if(limits_hit_external.value[jointThrIdx[i]] == 1 && msg.throttles[i] < 0){
      forcing_limit.value = true
    } else if (limits_hit_external.value[jointThrIdx[i]] == 2 && msg.throttles[i] > 0) {
      forcing_limit.value = true
    }
  }

  if(forcing_limit.value){
  vibrationActuator.value.playEffect('dual-rumble', {
    startDelay: 0,
    duration: 100,
    weakMagnitude: 0.1,
    strongMagnitude: 0,})
  }
})

// watch(armMessage, (msg: unknown) => {
// if (!msg || typeof msg !== 'object') return

//   if ('type' in msg && msg.type === "arm_state"){
//     const typedMsg = msg as ControllerStateMessage
//     for(let i = 0; i < 6; ++i){
//       limits_hit_external.value[i] = typedMsg.limits_hit[i]
//     }
//   }

  // if ('type' in msg && msg.type === "arm_throttle_command"){
  //   const typedMessage = msg as ThrottleMessage
  //   forcing_limit.value = false;

  //   // Find what index is what joint. Only done first time thrust input is recieved
  //   if(jointThrIdx.length == 0){
  //     for(let i = 0; i < typedMessage.names.length; ++i){
  //       let currName = typedMessage.names[i]
  //       switch(currName){
  //         case "joint_a":
  //           jointThrIdx.push(0);
  //           break;
  //         case "joint_b":
  //           jointThrIdx.push(1);
  //           break;
  //         case "joint_c":
  //           jointThrIdx.push(2);
  //           break;
  //         case "joint_de_pitch":
  //           jointThrIdx.push(3);
  //           break;
  //         case "joint_de_roll":
  //           jointThrIdx.push(4);
  //           break;
  //         case "gripper":
  //           jointThrIdx.push(5);
  //           break;
  //         case "cam":
  //           jointThrIdx.push(6);
  //           break;
  //         default:
  //       }
  //     }
  //   }

  //   // Determine if limits hit
  //   for(let i = 0; i < jointThrIdx.length; ++i){
  //     if(limits_hit_external.value[jointThrIdx[i]] == 1 && typedMessage.throttles[i] < 0){
  //       forcing_limit.value = true
  //     } else if (limits_hit_external.value[jointThrIdx[i]] == 2 && typedMessage.throttles[i] > 0) {
  //       forcing_limit.value = true
  //     }
  //   }
  // }

  // if(forcing_limit.value){
  //   vibrationActuator.value.playEffect('dual-rumble', {
  //     startDelay: 0,
  //     duration: 100,
  //     weakMagnitude: 0.1,
  //     strongMagnitude: 0,})
  // }
// })
</script>
