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


<style scoped>
.wrap {
    display: flex;
    align-items: center;
    height: 100%;
    width: 100%;
}
</style>
