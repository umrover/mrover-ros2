import { ref, computed, watch, nextTick, onMounted, type Ref } from 'vue'
import type WaypointItem from '@/components/AutonWaypointItem.vue'
import type VimBar from '@/components/VimBar.vue'
import { useWaypointKeyboard } from '@/composables/useWaypointKeyboard'

interface VimEditorRefs {
  editorRef: Ref<HTMLElement | null>
  storeItemRefs: Record<number, InstanceType<typeof WaypointItem>>
  commandBar: Ref<InstanceType<typeof VimBar> | null>
}

const LS_VIM = 'autonEditor.vimEnabled'

export function useVim({ editorRef, storeItemRefs, commandBar }: VimEditorRefs) {
  const keyboard = useWaypointKeyboard()

  const vimEnabled = ref(localStorage.getItem(LS_VIM) === 'true')
  const editorFocused = ref(false)
  const modalOpen = ref(false)

  const visualSelectedCount = computed(() => {
    return keyboard.storeSelectedIndices.value.size + keyboard.executionSelectedIndices.value.size
  })

  onMounted(() => {
    if (vimEnabled.value) {
      editorRef.value?.focus()
    }
  })

  function openStoreEditModal() {
    if (keyboard.focusedColumn.value !== 'store') return
    const item = storeItemRefs[keyboard.storeIndex.value]
    if (!item) return
    item.openEditModal()
    modalOpen.value = true
    const unwatch = watch(() => item.isOpen, (open) => {
      if (!open) {
        modalOpen.value = false
        unwatch()
        nextTick(() => editorRef.value?.focus())
      }
    })
  }

  keyboard.setOnEnter(openStoreEditModal)

  watch(() => keyboard.scrollKey.value, () => {
    if (!vimEnabled.value || !editorFocused.value) return
    nextTick(() => {
      const el = editorRef.value?.querySelector('.kbd-highlighted')
      if (el) {
        el.scrollIntoView({ block: 'nearest', behavior: 'smooth' })
      }
    })
  })

  function toggleVim() {
    vimEnabled.value = !vimEnabled.value
    localStorage.setItem(LS_VIM, String(vimEnabled.value))
    if (vimEnabled.value) {
      editorRef.value?.focus()
    } else {
      keyboard.exitVisualMode()
      editorRef.value?.blur()
      editorFocused.value = false
    }
  }

  function handleKeydown(e: KeyboardEvent) {
    if (!vimEnabled.value) return
    if (commandBar.value?.active) return
    if (e.key === 'Tab') {
      e.preventDefault()
      return
    }
    if (e.key === ':') {
      e.preventDefault()
      commandBar.value?.open()
      return
    }
    keyboard.handleKeydown(e)
  }

  function handleVimCommand(cmd: string) {
    const item = storeItemRefs[keyboard.storeIndex.value] ?? null
    const isModalOpen = item?.isOpen

    switch (cmd) {
      case 'e':
        if (!isModalOpen) openStoreEditModal()
        break
      case 'w':
        if (isModalOpen) item?.saveEdit()
        break
      case 'q':
        if (isModalOpen) item?.closeEditModal()
        break
      case 'wq':
      case 'x':
        if (isModalOpen) item?.saveAndClose()
        break
    }

    nextTick(() => {
      if (!modalOpen.value) editorRef.value?.focus()
    })
  }

  function restoreFocus() {
    nextTick(() => {
      if (modalOpen.value) {
        const backdrop = document.querySelector('.modal-backdrop') as HTMLElement | null
        backdrop?.focus()
      } else {
        editorRef.value?.focus()
      }
    })
  }

  function handleBlur(e: FocusEvent) {
    if (!vimEnabled.value) {
      editorFocused.value = false
      return
    }
    if (modalOpen.value || commandBar.value?.active) return
    const wrapper = editorRef.value
    if (wrapper && e.relatedTarget instanceof Node && wrapper.contains(e.relatedTarget)) {
      return
    }
    nextTick(() => wrapper?.focus())
  }

  return {
    keyboard,
    vimEnabled,
    editorFocused,
    modalOpen,
    visualSelectedCount,
    toggleVim,
    handleKeydown,
    handleVimCommand,
    restoreFocus,
    handleBlur,
  }
}
