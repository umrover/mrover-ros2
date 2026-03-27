import { ref, onMounted, onUnmounted } from 'vue'

export function useModal() {
  const isOpen = ref(false)

  function show() {
    isOpen.value = true
    document.body.style.overflow = 'hidden'
  }

  function hide() {
    isOpen.value = false
    document.body.style.overflow = ''
  }

  function onKeydown(e: KeyboardEvent) {
    if (e.key === 'Escape' && isOpen.value) {
      hide()
    }
  }

  onMounted(() => {
    document.addEventListener('keydown', onKeydown)
  })

  onUnmounted(() => {
    document.removeEventListener('keydown', onKeydown)
    document.body.style.overflow = ''
  })

  return { isOpen, show, hide }
}
