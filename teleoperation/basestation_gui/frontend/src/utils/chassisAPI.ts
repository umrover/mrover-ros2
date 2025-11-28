import type { APIResponse } from './apiTypes'

const API_BASE = '/api'

export const chassisAPI = {
  async startPanorama(): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/chassis/panorama/start/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  },

  async stopPanorama(): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/chassis/panorama/stop/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  },

  async adjustGimbal(joint: 'pitch' | 'yaw', adjustment: number, absolute: boolean = false): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/chassis/gimbal/adjust/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ joint, adjustment, absolute })
    })
    return response.json()
  },

  async setFunnelServo(name: string[], position: number[]): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/chassis/sp_funnel_servo/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, position })
    })
    return response.json()
  }
}
