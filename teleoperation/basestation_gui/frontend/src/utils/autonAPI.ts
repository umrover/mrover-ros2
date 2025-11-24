import type {
  AutonEnableResponse,
  TeleopEnableResponse,
  AutonEnableWaypoint
} from './apiTypes'

const API_BASE = '/api'

export const autonAPI = {
  async enable(enabled: boolean, waypoints: AutonEnableWaypoint[]): Promise<AutonEnableResponse> {
    const response = await fetch(`${API_BASE}/auton/enable/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled, waypoints })
    })

    if (!response.ok) {
      const data = await response.json()
      const error_message = data["message"]
      console.log(error_message)
      return { status: 'error', message: `${error_message}` }
    }

    return response.json()
  },

  async enableTeleop(enabled: boolean): Promise<TeleopEnableResponse> {
    const response = await fetch(`${API_BASE}/teleop/enable/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled })
    })

    if (!response.ok) {
      const data = await response.json()
      const error_message = data["message"]
      console.log(error_message)
      return { status: 'error', message: `HTTP ${response.status}: ${response.statusText}... ${error_message}` }
    }

    return response.json()
  }
}
