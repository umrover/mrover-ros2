<template>
    <h3 class="text-center">HexHub Controls</h3>
    <!-- CW / CCW toggle -->
    <div class="d-flex justify-content-center align-items-center my-3 gap-2">
      <input
        type="radio"
        class="btn-check"
        id="cw"
        value="true"
        v-model="isClockwise"
        autocomplete="off"
        @change="emitOrientation"
      />
      <label
        class="btn"
        :class="isClockwise === 'true' ? 'btn-primary' : 'btn-outline-primary'"
        for="cw"
      >Clockwise</label>

      <input
        type="radio"
        class="btn-check"
        id="ccw"
        value="false"
        v-model="isClockwise"
        autocomplete="off"
        @change="emitOrientation"
      />
      <label
        class="btn"
        :class="isClockwise === 'false' ? 'btn-primary' : 'btn-outline-primary'"
        for="ccw"
      >Counter-Clockwise</label>
    </div>

      <!-- site selection -->
      <div class="grid">
        <div v-for="name, index in siteList" :key="index" class="col-4">
          <div class="form-check d-flex justify-content-center align-items-center" >
            <input
              v-model="currentSite"
              class="btn-check"
              type="radio"
              :id="'Site' + index"
              :value="index"
              @change="emitSite"
            />
            <label class="btn btn-outline-secondary my-2" :for="'Site' + index">{{ name }}</label>
          </div>
        </div>
      </div>
  </template>

  <script lang="ts">
  import { mapState, mapActions } from 'vuex';
  export default {
    data() {
      return {
        currentSite: 0,
        siteList: ["Sample A", "Sample B", "Sample Cache", "Empty Soil Deposit"],
        isClockwise: true, // hexhub will rotate clockwise by default
      };
    },
    computed: {
      ...mapState('websocket', ['message']),
    },
    methods: {
      ...mapActions('websocket', ['sendMessage']),
      emitSite() {
        this.$emit('selectSite', this.currentSite); 
      },
      emitOrientation() {
        const isCCW = this.isClockwise === 'false'; // If not clockwise, it’s counterclockwise
        this.$emit('orientation', isCCW);
      }
    },

    emits: ['selectSite'],
  };
  </script>

  <style scoped>
  .btn{
    width: auto; 
    transition: transform 0.2s, box-shadow 0.2s;
  }

  .btn:hover {
    transform: translateY(-4px);
    box-shadow: 0 6px 10px rgba(0, 0, 0, 0.2);
  }

  .grid {
  display: grid;
  /* grid-gap: 0px; */
  grid-template-columns: repeat(2, auto);
  font-family: sans-serif;
  height: auto;
  align-items: center;
  justify-items: center;
}
  </style>
    
