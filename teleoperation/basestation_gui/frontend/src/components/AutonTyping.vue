<!-- textbox
submit button for textbox
enter 3-6 character string (don't need bounds check)
send submission thru websocket to backend
once it reaches the backend, create a rosaction (consumers.py)
make an autonTyping rosaction
create instance of rosaction in callback area -->

<template>
  <form>
    <div class="form-group col-md-4">
          <input 
            v-model="typingMessage"
            type="text"
            class="form-control"
            id="autonTyping"
            placeholder="Message"
            maxlength="6"
          />
          <small id="autonTypingMission" class="form-text text-muted"></small>
        </div>
    <div class="col-auto">
      <span id="typingHelp" class="form-text">
        Must be 3-6 characters long.
      </span>
    </div>
    <button class="btn btn-primary custom-btn" @click="submitMessage()">Submit</button>
  </form>
</template>

<script lang="ts">
import { mapMutations, mapGetters, mapActions, mapState } from 'vuex'


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
    }
  },

  watch: {
    // message(msg) {
    //   if (msg.type == '') { // TODO: where message needs to be assigned?
    //   }
    // },

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
      //TODO: send the code via ROS action
      if (!this.codeSent) {
        this.sendMessage({
          type: 'code',
          code: this.typingMessage
        })
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