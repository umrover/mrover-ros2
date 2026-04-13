import type { RAModeResponse, StowResponse, StowPosition, StowConfigResponse } from './apiTypes'
import { apiFetch } from './apiFetch'

export const armAPI = {
  setRAMode(mode: string): Promise<RAModeResponse> {
    return apiFetch('/arm/ra_mode/', {
      method: 'POST',
      body: JSON.stringify({ mode })
    })
  },

  stowArm(): Promise<StowResponse> {
    return apiFetch('/arm/stow/', {
      method: 'POST'
    })
  },

  getStowConfig(): Promise<StowConfigResponse> {
    return apiFetch('/arm/stow/config/', {
      method: 'GET'
    })
  },

  captureStowPose(): Promise<StowConfigResponse> {
    return apiFetch('/arm/stow/capture/', {
      method: 'POST'
    })
  },

  saveStowConfig(data: StowPosition): Promise<StowConfigResponse> {
    return apiFetch('/arm/stow/config/', {
      method: 'POST',
      body: JSON.stringify(data)
    })
  },

  resetStowConfig(): Promise<StowConfigResponse> {
    return apiFetch('/arm/stow/config/reset/', {
      method: 'POST'
    })
  }
}
