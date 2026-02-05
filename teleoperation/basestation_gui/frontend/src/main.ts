import { createApp } from 'vue'
import App from './App.vue'
import router from './router'
import { store } from './store'

import '@fontsource/jetbrains-mono'
import 'bootstrap-icons/font/bootstrap-icons.css'
import './app.scss'

const app: App<Element> = createApp(App)
app.use(router).use(store).mount('#app')