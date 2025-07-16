import { createApp } from 'vue'
import App from './App.vue'
import router from './router'
import VueKonva from 'vue-konva';

createApp(App).use(router).use(VueKonva).mount('#app')