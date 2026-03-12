import { defineStore } from 'pinia'
import { ref, watch } from 'vue'

export const themes = ['light', 'dark', 'high-contrast-light', 'high-contrast-dark', 'dont-click-me'] as const
export type Theme = (typeof themes)[number]

const STORAGE_KEY = 'theme_preference'

function loadFromStorage(): Theme {
  try {
    const stored = localStorage.getItem(STORAGE_KEY)
    if (stored === 'high-contrast') {
      return 'high-contrast-dark'
    }
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
  const bsTheme = theme === 'dark' || theme === 'high-contrast-dark' ? 'dark' : 'light'
  document.documentElement.setAttribute('data-bs-theme', bsTheme)
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
