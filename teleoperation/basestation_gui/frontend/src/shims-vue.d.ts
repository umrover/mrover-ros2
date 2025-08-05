import { Store } from 'vuex'
import State from './store/types.ts'

declare module '@vue/runtime-core' {
  interface ComponentCustomProperties {
    $store: Store<State>
  }
}
