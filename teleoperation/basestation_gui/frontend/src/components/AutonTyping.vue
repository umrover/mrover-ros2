<template>
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
        class="btn btn-sm btn-outline-control"
        data-testid="pw-typing-submit"
        :disabled="typingMessage.length < 3 || codeSent"
        @click.prevent="submitCode()"
      >
        Send
      </button>
      <button
        class="btn btn-sm btn-outline-danger"
        data-testid="pw-typing-cancel"
        :disabled="!codeSent"
        @click.prevent="cancelCode()"
      >
        Cancel
      </button>
      <span class="spacer"></span>
      <span class="yaw-label">YAW</span>
      <span class="data-value" v-html="formatNumber(yawDegrees, 3, 1, true)"></span><span class="data-unit">&deg;</span>
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
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { formatNumber } from '@/utils/formatNumber'

interface KeyboardYawMessage {
  type: 'keyboard_yaw'
  yaw: number
}

interface TypingFeedbackMessage {
  type: 'typing_feedback'
  current_index: number
  current_state: string
}

interface TypingCancelledMessage {
  type: 'typing_cancelled'
}

const websocketStore = useWebsocketStore()

const typingMessage = ref('')
const codeSent = ref(false)
const currentIndex = ref(0)
const currentState = ref('')
const letterStates = ref<string[]>(Array(6).fill('notTyped'))
const yawDegrees = ref(0)

onMounted(() => {
  websocketStore.setupWebSocket('auton')
})

onBeforeUnmount(() => {
  websocketStore.closeWebSocket('auton')
})

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
  }
})

websocketStore.onMessage<KeyboardYawMessage>('auton', 'keyboard_yaw', (msg) => {
  yawDegrees.value = msg.yaw * (180 / Math.PI)
})

websocketStore.onMessage<TypingCancelledMessage>('auton', 'typing_cancelled', () => {
  codeSent.value = false
  typingMessage.value = ''
  letterStates.value = Array(6).fill('notTyped')
})

function submitCode() {
  websocketStore.sendMessage('auton', {
    type: 'code',
    code: typingMessage.value,
  })
  codeSent.value = true
  letterStates.value = new Array(typingMessage.value.length).fill('notTyped')
}

function cancelCode() {
  websocketStore.sendMessage('auton', {
    type: 'code',
    code: 'cancel',
  })
  codeSent.value = false
  typingMessage.value = ''
  letterStates.value = Array(6).fill('notTyped')
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
  if (!codeSent.value) return 'grey-cell' // grey when not sent

  return {
    'grey-cell': state === 'grey',
    'typed-cell': state === 'typed',
    'in-progress-cell': state === 'inProgress',
    'not-typed-cell': state === 'notTyped',
  }
}
</script>

<style scoped>
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
}

.feedback-table {
  border-collapse: collapse;
  width: 100%;
}

.feedback-table td {
  height: clamp(32px, 2.5vw, 48px);
  font-size: 1.25rem;
  font-weight: 700;
  text-align: center;
  line-height: 1;
}

.grey-cell {
  color: var(--text-muted);
  background-color: var(--view-bg);
}

.typed-cell {
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
}
</style>
