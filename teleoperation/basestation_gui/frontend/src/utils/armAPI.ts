import type { RAModeResponse } from './apiTypes'
import { apiFetch } from './apiFetch'
import { idText } from 'typescript'

export const armAPI = {
  setRAMode(mode: string): Promise<RAModeResponse> {
    return apiFetch('/arm/ra_mode/', {
      method: 'POST',
      body: JSON.stringify({ mode })
    })
  },

  stowArm(): Promise<true> {
    return apiFetch('/arm/stow/', {
      method: 'POST',
    })
  }

}
