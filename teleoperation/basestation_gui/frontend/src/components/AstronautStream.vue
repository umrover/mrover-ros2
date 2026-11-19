 <template>
  <div>
    <div
    class="h-full flex flex-row select-none"
    >
      <div class="flex-col justify-start px-4 py-3">
        <i class="bi bi-volume-up text-xl" style="font-size: 48px;"></i>
      </div>
      <div class="astro">
        <h4 class="component-header flex-col items-align justify-center">
          <span class="flex-col gap-1">
            Astronaut
          </span>
        </h4>
      </div>

      <div class="flex-col px-5 py-5">
        <div class="stream">
          <div class="border-2 border-theme rounded px-2 flex flex-col justify-center font-mono text-sm"
            :class="0 && 0 === null ? '' : 'text-muted'">
            <div>
              <span class="text-muted font-semibold">RX</span>
              <span :class="0 && 0 === null ? 'text-danger' : ''">
                <span v-html="formatNumber(0, 4, 0)"></span><span
                  :class="0 && 0 === null ? '' : 'text-muted'">ms</span>
              </span>
            </div>
          </div>
        </div>

        <div class="stream">
          <div class="border-2 border-theme rounded px-2 flex flex-col justify-center font-mono text-sm"
            :class="0 && 0 === null ? '' : 'text-muted'">
            <div>
              <span class="text-muted font-semibold">RX</span>
              <span :class="0 && 0 === null ? 'text-danger' : ''">
                <span v-html="formatNumber(0, 4, 0)"></span><span
                  :class="0 && 0 === null ? '' : 'text-muted'">ms</span>
              </span>
            </div>
          </div>
        </div>
      </div>

    </div>

    

  </div>
</template>

<script lang="ts" setup>
import { useControllerMessage } from '@/composables/useControllerState'
import { useModal } from '@/composables/useModal'

import { formatNumber } from '@/utils/formatNumber';


const modal = useModal()

const { data } = useControllerMessage({
  topic: 'drive',
  messageType: 'drive_state',
})

</script>

<style scoped>

.astro{
  position: relative;
  display: flex;
  justify-content: center;
  padding-right: 5px;
  left: 170px;
}

.stream{
  position: relative;
  display: flex;
  justify-content: center;
  padding-top: 5px;
  left: 50px;
}


</style>
