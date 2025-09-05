import autonomy from './modules/autonomy'
import cameras from './modules/cameras'
import erd from './modules/erd'
import map from './modules/map'
import websocket from './modules/websocket'
import { createStore } from 'vuex'

export const store = createStore({
  modules: {
    autonomy,
    cameras,
    erd,
    map,
    websocket
  }
})
