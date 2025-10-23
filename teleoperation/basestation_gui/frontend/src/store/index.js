import autonomy from './modules/autonomy'
import cameras from './modules/cameras'
import erd from './modules/erd'
import websocket from './modules/websocket'
import notifications from './modules/notifications'
import { createStore } from 'vuex'

export const store = createStore({
  modules: {
    autonomy,
    cameras,
    erd,
    websocket,
    notifications
  }
})
