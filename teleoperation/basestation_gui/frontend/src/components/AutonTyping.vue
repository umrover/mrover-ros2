<!-- textbox
submit button for textbox
enter 3-6 character string (don't need bounds check)
send submission thru websocket to backend
once it reaches the backend, create a rosaction (consumers.py)
make an autonTyping rosaction
create instance of rosaction in callback area -->

<template>
  <div class="auton-typing-input">
    <h4>Autonomous Typing Task</h4>
    <form>
      <div class="form-group col-md-4">
            <input 
              v-model="typingMessage"
              type="text"
              class="form-control"
              id="autonTyping"
              placeholder="Message"
              maxlength="6"
              required
            />
            <small id="autonTypingMission" class="form-text text-muted"></small>
          </div>
      <div class="col-auto">
        <span id="typingHelp" class="form-text">
          Must be 3-6 characters long.
        </span>
      </div>
      <button v-if="codeSent === false" class="btn btn-primary custom-btn" :disabled="typingMessage.length < 3" @click.prevent="submitMessage()">Submit</button>
      <!-- TODO: add a separate function to cancel current message -->
      <button v-if="codeSent === true" class="btn btn-primary custom-btn bg-danger" @click.prevent="submitMessage()">Cancel</button>
    </form>
  </div>
  <div class="feedback-section">
    <h4>Feedback</h4>
    <table>
      <thead>
        <tr>
          <th>Key</th>
          <th>State</th>
        </tr>
      </thead>
      <tbody>
        <!-- add this back into the tr tag underneath: v-for="(state, key) in feedback" :key="key" -->
        <tr>
          <!-- <td>{{ key }}</td>
          <td>{{ state }}</td> -->
          <!-- TODO: Remove this (only used for example purposes) -->
          <td>{{currentKey}}</td>
          <td>{{currentState}}</td>
        </tr>
      </tbody>
    </table>
  </div>
</template>

<script lang="ts">
import { mapActions, mapState } from 'vuex'


export default {
  components: {
  },

  props: {
    mission: {
      type: String, // {'auton'}
      required: true
    }
  },
  data() {
    return {
      typingMessage: '',
      codeSent: false,
      currentKey: '',
      currentState: '',
    }
  },

  watch: {
    message(msg) {
      if (msg.type == 'keyAction') { // TODO: where message needs to be assigned?
        this.currentKey = msg.currentKey;
        this.currentState = msg.currentState;
      }
    },

    // window.setTimeout(() => {
    //   // Timeout so websocket will be initialized
    //   this.sendMessage({ type: 'get_auton_typing_message' })
    // }, 250),
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    submitMessage: function() {
      console.log("sending a message")
      //TODO: send the code via ROS action
      if (!this.codeSent) {
        this.sendMessage({
          type: 'code',
          code: this.typingMessage
        })
        this.codeSent = !this.codeSent;
      } 
      else {
      // Canceling the current message
      this.sendMessage({
        type: 'code',
        code: 'cancel'
      });
      console.log("sending cancel message")
      this.codeSent = false;  // Reset the button to "Submit"
      this.typingMessage = ''; // Optionally, clear the input field
    }

    },
  }
}
</script>

<style scoped>
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
</style>