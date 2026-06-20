import { defineStore } from 'pinia'
import { ref, watch } from 'vue'

export const themes = ['light', 'dark'] as const
export type Theme = (typeof themes)[number]

const STORAGE_KEY = 'theme_preference'

function loadFromStorage(): Theme {
  try {
    const stored = localStorage.getItem(STORAGE_KEY)
    if (stored && themes.includes(stored as Theme)) {
      return stored as Theme
    }
  } catch {}
  return 'light'
}

function saveToStorage(theme: Theme) {
  localStorage.setItem(STORAGE_KEY, theme)
}

function applyTheme(theme: Theme) {
  document.documentElement.setAttribute('data-theme', theme)
}

export const useThemeStore = defineStore('theme', () => {
  const currentTheme = ref<Theme>(loadFromStorage())

  watch(currentTheme, (theme) => {
    saveToStorage(theme)
    applyTheme(theme)
  })

  function setTheme(theme: Theme) {
    currentTheme.value = theme
  }

  function initTheme() {
    applyTheme(currentTheme.value)
  }

  return {
    currentTheme,
    setTheme,
    initTheme
  }
})
