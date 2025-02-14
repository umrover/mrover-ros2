import { Store } from 'vuex'
import { ComponentCustomProperties } from 'vue'

declare module '@vue/runtime-core' {
  interface ComponentCustomProperties {
    $store: Store<any>
  }
}
