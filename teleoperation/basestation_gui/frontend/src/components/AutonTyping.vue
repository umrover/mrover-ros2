<!-- textbox
submit button for textbox
enter 3-6 character string (don't need bounds check)
send submission thru websocket to backend
once it reaches the backend, create a rosaction (consumers.py)
make an autonTyping rosaction
create instance of rosaction in callback area -->

<template>
  <div class="auton-typing-container">
    <!-- left col -->
    <div class="column left">
      <h5>Typing Input</h5>
      <form>
        <div class="form-group">
          <input
            v-model="typingMessage"
            type="text"
            class="form-control"
            id="autonTyping"
            placeholder="Message"
            maxlength="6"
            required
            :disabled="codeSent"
          />
        </div>
        <span class="form-text">Must be 3-6 characters long.</span>

        <div class="button-group">
          <button
            v-if="!codeSent"
            class="btn btn-primary custom-btn"
            :disabled="typingMessage.length < 3"
            @click.prevent="submitMessage()"
          >
            Submit
          </button>
          <button
            v-if="codeSent"
            class="btn btn-danger custom-btn"
            @click.prevent="submitMessage()"
          >
            Cancel
          </button>
        </div>
      </form>
    </div>

    <div class="stacked-columns">
      <!-- middle col -->
      <div class="column middle">
        <h5>Feedback</h5>
        <table class="feedback-table">
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

      <!-- right col, needs implementation, ask for specifics -->
      <div class="column right">
        <h5>Planar Alignment</h5>
        <div class="wrap border border-2 rounded p-1">
          <h6 class="m-0 p-0 font-monospace text-center">
            0 degrees
          </h6>
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
  gap: 40px;
  justify-content: center;
  align-items: start;
  text-align: center;
  width: 100%;
  padding: 2px 5px 2px 5px;
}

.column {
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  width: 100%;
}

.custom-btn {
  width: 150px;
  height: 50px;
}

.info {
  height: 200px;
  overflow-y: auto;
}

.percent {
  font-size: large;
}
.feedback-table {
  border-collapse: collapse;
  margin-top: 10px;
}

.feedback-table td {
  width: 40px;
  height: 40px;
  text-align: center;
  font-size: 24px;
  font-weight: bold;
  border: 1px solid #ccc;
}

.grey-cell {
  background-color: #d3d3d3;
  color: black;
}

.typed-cell {
  background-color: green;
  color: white;
}

.in-progress-cell {
  background-color: orange;
  color: white;
}

.not-typed-cell {
  background-color: red;
  color: white;
}

.stacked-columns {
  display: flex;
  flex-direction: column;
  gap: 20px;
}
</style>
