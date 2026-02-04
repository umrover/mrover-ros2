import type {
  AutonEnableResponse,
  TeleopEnableResponse,
  AutonEnableWaypoint
} from './apiTypes'
import { apiFetch } from './apiFetch'

export const autonAPI = {
  enable(enabled: boolean, waypoints: AutonEnableWaypoint[]): Promise<AutonEnableResponse> {
    return apiFetch('/enable_auton/', {
      method: 'POST',
      body: JSON.stringify({ enabled, waypoints })
    })
  },

  enableTeleop(enabled: boolean): Promise<TeleopEnableResponse> {
    return apiFetch('/enable_teleop/', {
      method: 'POST',
      body: JSON.stringify({ enabled })
    })
  }
}
