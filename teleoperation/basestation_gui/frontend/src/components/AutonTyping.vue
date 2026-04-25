<template>
<<<<<<< HEAD
  <div class="grid grid-cols-2 gap-6 w-full p-2 items-start mx-0">
    <div class="flex flex-col items-center text-center">
      <h4 class="component-header mb-2">Typing Input</h4>
      <form>
        <div class="form-group">
          <input
            v-model="typingMessage"
            type="text"
            class="cmd-form-control cmd-input"
            id="autonTyping"
            data-testid="pw-typing-input"
            placeholder="Message"
            maxlength="6"
            required
            :disabled="codeSent"
          />
        </div>
        <span class="typing-hint">Must be 3-6 characters long.</span>

        <div class="flex justify-center">
          <button
            v-if="!codeSent"
            class="cmd-btn cmd-btn-sm cmd-btn-outline-control typing-btn"
            data-testid="pw-typing-submit"
            :disabled="typingMessage.length < 3"
            @click.prevent="submitMessage()"
          >
            Submit
          </button>
          <button
            v-if="codeSent"
            class="cmd-btn cmd-btn-sm cmd-btn-outline-secondary typing-btn"
            @click.prevent="submitMessage()"
          >
            Cancel
          </button>
        </div>
      </form>
    </div>

    <div class="flex flex-col gap-4">
      <div class="flex flex-col items-center text-center w-full">
        <h4 class="component-header mb-2">Feedback</h4>
        <table class="feedback-table" data-testid="pw-typing-feedback">
          <tbody>
            <tr>
              <td
                v-for="index in 6"
                :key="index"
                :class="getLetterClass(letterStates[index - 1] ?? 'grey')"
              >
                {{ typingMessage[index - 1] ?? '_' }}
              </td>
            </tr>
          </tbody>
        </table>
      </div>

      <div class="flex flex-col items-center text-center w-full">
        <h4 class="component-header mb-2">Planar Alignment</h4>
        <div class="flex items-baseline justify-center gap-1 p-2 rounded bg-theme-view">
          <span class="cmd-data-value">{{ planarAngle }}</span>
          <span class="cmd-data-unit">degrees</span>
        </div>
      </div>
=======
  <div class="typing-panel">
    <div class="typing-row">
      <input
        v-model="typingMessage"
        type="text"
        class="typing-input"
        data-testid="pw-typing-input"
        maxlength="6"
        :disabled="codeSent"
      />
      <button
        v-if="!codeSent"
        class="btn btn-sm btn-outline-control"
        data-testid="pw-typing-submit"
        :disabled="typingMessage.length < 3"
        @click.prevent="submitMessage()"
      >
        Send
      </button>
      <button
        v-if="codeSent"
        class="btn btn-sm btn-outline-danger"
        data-testid="pw-typing-cancel"
        @click.prevent="submitMessage()"
      >
        Cancel
      </button>
      <span class="spacer"></span>
      <span class="yaw-label">YAW</span>
      <span class="data-value">{{ yawAngle.toFixed(3) }}</span><span class="data-unit">rad</span>
>>>>>>> origin/main
    </div>
    <table class="feedback-table" data-testid="pw-typing-feedback">
      <tbody>
        <tr>
          <td
            v-for="index in 6"
            :key="index"
            :class="getLetterClass(letterStates[index - 1] ?? 'grey')"
          >
            {{ typingMessage[index - 1] ?? '_' }}
          </td>
        </tr>
      </tbody>
    </table>
  </div>
</template>

<script lang="ts" setup>
<<<<<<< HEAD
import { ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'
=======
import { ref, onMounted, onBeforeUnmount } from 'vue'
>>>>>>> origin/main
import { useWebsocketStore } from '@/stores/websocket'

interface TypingFeedbackMessage {
  type: 'typing_feedback'
  current_index: number
  current_state: string
}

interface TypingCancelledMessage {
  type: 'typing_cancelled'
}

interface KeyboardYawMessage {
  type: 'keyboard_yaw'
  yaw: number
}

const websocketStore = useWebsocketStore()

const typingMessage = ref('')
const codeSent = ref(false)
const currentIndex = ref(0)
const currentState = ref('')
const letterStates = ref<string[]>(Array(6).fill('notTyped'))
<<<<<<< HEAD
const planarAngle = ref(0)

onMounted(() => {
  websocketStore.setupWebSocket('auton')
  websocketStore.setupWebSocket('arm')

  interface IKFeedbackMessage {
    type: 'ik_feedback'
    pos: { x: number; y: number; z: number }
  }

  websocketStore.onMessage<IKFeedbackMessage>('arm', 'ik_feedback', msg => {
  const p: any = msg.pos ?? (msg as any).position ?? [0, 0]
  const x = Number(p.x ?? p[0] ?? 0)
  const y = Number(p.y ?? p[1] ?? 0)
  if (!isFinite(x) || !isFinite(y)) return
  planarAngle.value = Math.round((Math.atan2(y, x) * 180) / Math.PI)
  })
})

onBeforeUnmount(() => {
  websocketStore.closeWebSocket('auton')
  websocketStore.closeWebSocket('arm')
})
=======
const yawAngle = ref(0)
>>>>>>> origin/main

onMounted(() => {
  websocketStore.setupWebSocket('auton')
})

onBeforeUnmount(() => {
  websocketStore.closeWebSocket('auton')
})

<<<<<<< HEAD
  if ('type' in msg && msg.type === 'typing_feedback') {
    const typedMsg = msg as TypingFeedbackMessage
    currentIndex.value = typedMsg.current_index
    currentState.value = typedMsg.current_state

    if (typedMsg.current_state === 'complete') {
      for (let i = 0; i < typingMessage.value.length; i++) {
        letterStates.value[i] = 'typed'
      }
      codeSent.value = false
      typingMessage.value = ''
    } else {
      updateLetterStates()
    }
  } else if ('type' in msg && msg.type === 'typing_cancelled') {
    codeSent.value = false
    typingMessage.value = ''
    letterStates.value = Array(6).fill('notTyped')
=======
websocketStore.onMessage<TypingFeedbackMessage>('auton', 'typing_feedback', (msg) => {
  currentIndex.value = msg.current_index
  currentState.value = msg.current_state

  if (msg.current_state === 'complete') {
    for (let i = 0; i < typingMessage.value.length; i++) {
      letterStates.value[i] = 'typed'
    }
    codeSent.value = false
    typingMessage.value = ''
  } else {
    updateLetterStates()
>>>>>>> origin/main
  }
})

websocketStore.onMessage<KeyboardYawMessage>('auton', 'keyboard_yaw', (msg) => {
  yawAngle.value = msg.yaw
})

websocketStore.onMessage<TypingCancelledMessage>('auton', 'typing_cancelled', () => {
  codeSent.value = false
  typingMessage.value = ''
  letterStates.value = Array(6).fill('notTyped')
})

function submitMessage() {
  if (!codeSent.value) {
    websocketStore.sendMessage('auton', {
      type: 'code',
      code: typingMessage.value,
    })
    codeSent.value = true
    letterStates.value = new Array(typingMessage.value.length).fill('notTyped')
  } else {
    websocketStore.sendMessage('auton', {
      type: 'code',
      code: 'cancel',
    })
    codeSent.value = false
    typingMessage.value = ''
    letterStates.value = Array(6).fill('notTyped')
  }
}

function updateLetterStates() {
  if (!typingMessage.value) return
  for (let i = 0; i < typingMessage.value.length; i++) {
    if (i < currentIndex.value) {
      letterStates.value[i] = 'typed'
    } else if (i === currentIndex.value) {
      letterStates.value[i] = 'inProgress'
    } else {
      letterStates.value[i] = 'notTyped'
    }
  }
}

function getLetterClass(state: string) {
  if (!codeSent.value) return 'grey-cell'

  return {
    'grey-cell': state === 'grey',
    'typed-cell': state === 'typed',
    'in-progress-cell': state === 'inProgress',
    'not-typed-cell': state === 'notTyped',
  }
}
</script>

<style scoped>
<<<<<<< HEAD
.cmd-input {
  /* stylelint-disable-next-line declaration-property-value-disallowed-list */
  text-align: center;
  text-transform: uppercase;
  letter-spacing: 0.1em;
}

.typing-hint {
  /* stylelint-disable-next-line declaration-property-value-disallowed-list */
  display: block;
  margin: var(--cmd-gap-xs) 0 var(--cmd-gap-md);
  font-size: var(--cmd-font-xs);
  color: var(--text-muted);
}

.typing-btn {
  width: clamp(100px, 7vw, 140px);
  height: clamp(36px, 2.5vw, 48px);
=======
.typing-panel {
  display: flex;
  flex-direction: column;
  justify-content: center;
  gap: 0.5rem;
  height: 100%;
  padding: 0.5rem 0.75rem;
  text-transform: uppercase;
}

.typing-row {
  display: flex;
  align-items: center;
  gap: 0.375rem;
}

.typing-input {
  width: 5.5em;
  padding: 0.2rem 0.4rem;
  text-align: center;
  letter-spacing: 0.15em;
  font-weight: 600;
  font-size: 0.9rem;
  border: 1px solid var(--panel-border);
  border-radius: var(--radius-sm);
  background: var(--view-bg);
  color: var(--text-primary);
}

.spacer { flex: 1; }

.yaw-label {
  font-weight: 700;
  opacity: 0.6;
  letter-spacing: 0.06em;
  margin-right: 0.25rem;
>>>>>>> origin/main
}

.feedback-table {
  margin-top: var(--cmd-gap-md);
  border-collapse: collapse;
<<<<<<< HEAD
}

.feedback-table td {
  width: clamp(30px, 2vw, 44px);
  height: clamp(30px, 2vw, 44px);
  font-size: var(--cmd-font-xl);
  font-weight: 700;
  /* stylelint-disable-next-line declaration-property-value-disallowed-list */
  text-align: center;
=======
  width: 100%;
}

.feedback-table td {
  height: clamp(32px, 2.5vw, 48px);
  font-size: 1.25rem;
  font-weight: 700;
  text-align: center;
  line-height: 1;
>>>>>>> origin/main
}

.grey-cell {
  color: var(--text-muted);
  background-color: var(--view-bg);
}

.typed-cell {
<<<<<<< HEAD
  color: #fff;
  background-color: var(--cmd-status-ok);
}

.in-progress-cell {
  color: #fff;
  background-color: var(--cmd-status-warn);
}

.not-typed-cell {
  color: #fff;
  background-color: var(--cmd-status-error);
=======
  color: var(--text-on-status);
  background-color: var(--status-ok);
}

.in-progress-cell {
  color: var(--text-on-status);
  background-color: var(--status-warn);
}

.not-typed-cell {
  color: var(--text-on-status);
  background-color: var(--status-error);
>>>>>>> origin/main
}
</style>
