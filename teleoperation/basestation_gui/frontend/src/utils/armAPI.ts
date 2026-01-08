import type { RAModeResponse } from './apiTypes'

const API_BASE = '/api/arm'

export const armAPI = {
  async setRAMode(mode: string): Promise<RAModeResponse> {
    const response = await fetch(`${API_BASE}/ra_mode/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode })
    })
    return response.json()
  }
}
