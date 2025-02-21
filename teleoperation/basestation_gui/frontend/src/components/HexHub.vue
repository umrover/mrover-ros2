<template>
    <h3>HexHub Controls</h3>
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
      };
    },
    computed: {
      ...mapState('websocket', ['message']),
    },
    methods: {
      ...mapActions('websocket', ['sendMessage']),
      emitSite() {
        this.$emit('selectSite', this.currentSite); 
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
    
