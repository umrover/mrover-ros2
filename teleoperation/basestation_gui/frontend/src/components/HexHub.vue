<template>
    <div class="wrap">
        <h3>HexHub Options</h3>
        <div class="d-flex justify-content-center">
            <div v-for="h in hexHubOptions" :key="h" class='form-check mx-3'>
            <input
                v-model='hexHubPos'
                class='form-check-input'
                type='radio'
                :id="'hex'+h"
                :value='h'
            />
            <label class='form-check-label' :for="'hex'+h">{{h}}</label>
            </div>
        </div>
    </div>
</template>

<script lang='ts'>
export default {
  components: {
  },
  data() {
    return {
      hexHubPos: 0,
      hexHubOptions: ["Sample 1","Sample 2","Sample Cache","Empty Soil Deposit"]
    }
  },
}
</script>

<style scoped>
.wrap {
    display: flex;
    /* align-items: center; */
    height: 100%;
    width: 100%;
}
</style>
