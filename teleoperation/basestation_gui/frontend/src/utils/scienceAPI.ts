import type { APIResponse } from './apiTypes'

const API_BASE = '/api'

export const scienceAPI = {
  async setHeater(heaterName: string, enable: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/heater/${heaterName}/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable })
    })
    return response.json()
  },

  async setGearDiffPosition(position: number, isCounterclockwise: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/gear-diff/position/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ position, is_counterclockwise: isCounterclockwise })
    })
    return response.json()
  },

  async setAutoShutoff(enable: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/auto-shutoff/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable })
    })
    return response.json()
  },

  async setWhiteLEDs(site: string, enable: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/white-leds/${site}/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable })
    })
    return response.json()
  },

  async setLimitSwitch(enable: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/limit-switch/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable })
    })
    return response.json()
  }
}
