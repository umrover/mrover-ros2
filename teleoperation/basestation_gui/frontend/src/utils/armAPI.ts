import type { RAModeResponse } from './apiTypes'
import { apiFetch } from './apiFetch'

export const armAPI = {
  setRAMode(mode: string): Promise<RAModeResponse> {
    return apiFetch('/arm/ra_mode/', {
      method: 'POST',
      body: JSON.stringify({ mode })
    })
  },

  // TODO(stow): Add stowArm() method.
}
