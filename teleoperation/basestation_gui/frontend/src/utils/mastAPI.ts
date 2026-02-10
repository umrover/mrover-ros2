import type { APIResponse } from './apiTypes'

const API_BASE = '/api'

export const mastAPI = {
  async startPanorama(): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/mast/panorama/start/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  },

  async stopPanorama(): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/mast/panorama/stop/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  }
}
