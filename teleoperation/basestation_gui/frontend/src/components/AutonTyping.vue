<!-- textbox
submit button for textbox
enter 3-6 character string (don't need bounds check)
send submission thru websocket to backend
once it reaches the backend, create a rosaction (consumers.py)
make an autonTyping rosaction
create instance of rosaction in callback area -->

<template>
  <div class="auton-typing-container">
    <div class="d-flex flex-column align-items-center text-center w-100">
      <h4 class="component-header mb-2">Typing Input</h4>
      <form>
        <div class="form-group">
          <input
            v-model="typingMessage"
            type="text"
            class="form-control cmd-input border-2"
            id="autonTyping"
            placeholder="Message"
            maxlength="6"
            required
            :disabled="codeSent"
          />
        </div>
        <span class="typing-hint">Must be 3-6 characters long.</span>

        <div class="d-flex justify-content-center">
          <button
            v-if="!codeSent"
            class="btn btn-sm btn-outline-control border-2 typing-btn"
            :disabled="typingMessage.length < 3"
            @click.prevent="submitMessage()"
          >
            Submit
          </button>
          <button
            v-if="codeSent"
            class="btn btn-sm btn-outline-secondary border-2 typing-btn"
            @click.prevent="submitMessage()"
          >
            Cancel
          </button>
        </div>
      </form>
    </div>

    <div class="d-flex flex-column gap-3">
      <div class="d-flex flex-column align-items-center text-center w-100">
        <h4 class="component-header mb-2">Feedback</h4>
        <table class="feedback-table">
          <tbody>
            <tr>
              <td
                v-for="index in 6"
                :key="index"
                class="border-2"
                :class="getLetterClass(letterStates[index - 1] ?? 'grey')"
              >
                {{ typingMessage[index - 1] ?? '_' }}
              </td>
            </tr>
          </tbody>
        </table>
      </div>

      <div class="d-flex flex-column align-items-center text-center w-100">
        <h4 class="component-header mb-2">Planar Alignment</h4>
        <div class="d-flex align-items-baseline justify-content-center gap-1 p-2 rounded bg-theme-view">
          <span class="cmd-data-value">0</span>
          <span class="cmd-data-unit">degrees</span>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

interface TypingFeedbackMessage {
  type: 'typing_feedback'
  current_key: string
  current_state: string
}

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const typingMessage = ref('')
const codeSent = ref(false)
const currentKey = ref('')
const currentState = ref('')
const letterStates = ref<string[]>(Array(6).fill('notTyped'))

const autonMessage = computed(() => messages.value['auton'])

watch(autonMessage, (msg: unknown) => {
  if (!msg || typeof msg !== 'object') return

  if ('type' in msg && msg.type === 'typing_feedback') {
    const typedMsg = msg as TypingFeedbackMessage
    currentKey.value = typedMsg.current_key
    currentState.value = typedMsg.current_state
    updateLetterStates()
  }
})

function submitMessage() {
  console.log('sending a message')
  if (!codeSent.value) {
    // sending message
    websocketStore.sendMessage('auton', {
      type: 'code',
      code: typingMessage.value,
    })
    codeSent.value = true
    letterStates.value = new Array(typingMessage.value.length).fill('notTyped')
  } else {
    // cancel
    websocketStore.sendMessage('auton', {
      type: 'code',
      code: 'cancel',
    })
    codeSent.value = false
    typingMessage.value = '' // clear field
    letterStates.value = Array(6).fill('notTyped') // reset 6 empty cells
  }
}

function updateLetterStates() {
  if (!typingMessage.value) return
  for (let i = 0; i < typingMessage.value.length; i++) {
    if (i < currentKey.value.length) {
      letterStates.value[i] = 'typed' // green
    } else if (i === currentKey.value.length) {
      letterStates.value[i] = 'inProgress' // orange
    } else {
      letterStates.value[i] = 'notTyped' // red
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
.auton-typing-container {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: var(--cmd-gap-xl);
  justify-content: center;
  align-items: start;
  width: 100%;
  padding: var(--cmd-padding-sm);
}


.cmd-input {
  text-align: center;
  text-transform: uppercase;
  letter-spacing: 0.1em;
}

.typing-hint {
  font-size: var(--cmd-font-xs);
  color: var(--text-muted);
  display: block;
  margin: var(--cmd-gap-xs) 0 var(--cmd-gap-md);
}

.typing-btn {
  width: clamp(100px, 7vw, 140px);
  height: clamp(36px, 2.5vw, 48px);
}

.feedback-table {
  border-collapse: collapse;
  margin-top: var(--cmd-gap-md);
}

.feedback-table td {
  width: clamp(30px, 2vw, 44px);
  height: clamp(30px, 2vw, 44px);
  text-align: center;
  font-size: var(--cmd-font-xl);
  font-weight: 700;
}

.grey-cell {
  background-color: var(--view-bg);
  color: var(--text-muted);
}

.typed-cell {
  background-color: var(--cmd-status-ok);
  color: white;
}

.in-progress-cell {
  background-color: #f59e0b;
  color: white;
}

.not-typed-cell {
  background-color: var(--cmd-status-error);
  color: white;
}
</style>
