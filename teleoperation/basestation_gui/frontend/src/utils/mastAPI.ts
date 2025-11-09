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
  },

  async adjustGimbal(joint: 'pitch' | 'yaw', adjustment: number, absolute: boolean = false): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/mast/gimbal/adjust/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ joint, adjustment, absolute })
    })
    return response.json()
  }
}
