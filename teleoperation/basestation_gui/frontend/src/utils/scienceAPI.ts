import type { APIResponse } from './apiTypes'

const API_BASE = '/api'

export const scienceAPI = {
  async setGearDiffPosition(position: number, isCounterclockwise: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/gear-diff/position/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ position, is_counterclockwise: isCounterclockwise })
    })
    return response.json()
  }
}
