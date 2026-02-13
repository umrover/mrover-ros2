<!-- textbox
submit button for textbox
enter 3-6 character string (don't need bounds check)
send submission thru websocket to backend
once it reaches the backend, create a rosaction (consumers.py)
make an autonTyping rosaction
create instance of rosaction in callback area -->

<template>
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
          <span class="cmd-data-value">0</span>
          <span class="cmd-data-unit">degrees</span>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

interface TypingFeedbackMessage {
  type: 'typing_feedback'
  current_index: number
  current_state: string
}

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const typingMessage = ref('')
const codeSent = ref(false)
const currentIndex = ref(0)
const currentState = ref('')
const letterStates = ref<string[]>(Array(6).fill('notTyped'))

onMounted(() => {
  websocketStore.setupWebSocket('auton')
})

onBeforeUnmount(() => {
  websocketStore.closeWebSocket('auton')
})

const autonMessage = computed(() => messages.value['auton'])

watch(autonMessage, (msg: unknown) => {
  if (!msg || typeof msg !== 'object') return

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
}

.feedback-table {
  margin-top: var(--cmd-gap-md);
  border-collapse: collapse;
}

.feedback-table td {
  width: clamp(30px, 2vw, 44px);
  height: clamp(30px, 2vw, 44px);
  font-size: var(--cmd-font-xl);
  font-weight: 700;
  /* stylelint-disable-next-line declaration-property-value-disallowed-list */
  text-align: center;
}

.grey-cell {
  color: var(--text-muted);
  background-color: var(--view-bg);
}

.typed-cell {
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
}
</style>
