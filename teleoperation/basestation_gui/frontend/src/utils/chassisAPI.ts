import type { APIResponse } from './apiTypes'
import { apiFetch } from './apiFetch'

export const chassisAPI = {
  startPanorama(): Promise<APIResponse> {
    return apiFetch('/chassis/panorama/start/', { method: 'POST' })
  },

  stopPanorama(): Promise<APIResponse> {
    return apiFetch('/chassis/panorama/stop/', { method: 'POST' })
  },

  adjustGimbal(joint: 'pitch' | 'yaw', adjustment: number, absolute: boolean = false): Promise<APIResponse> {
    return apiFetch('/chassis/gimbal/adjust/', {
      method: 'POST',
      body: JSON.stringify({ joint, adjustment, absolute })
    })
  },

  setFunnelServo(name: string[], position: number[]): Promise<APIResponse> {
    return apiFetch('/chassis/sp_funnel_servo/', {
      method: 'POST',
      body: JSON.stringify({ name, position })
    })
  }
}
