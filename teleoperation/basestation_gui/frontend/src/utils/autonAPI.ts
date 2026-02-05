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
  },

  togglePurePursuit(enabled: boolean): Promise<TeleopEnableResponse> {
    return apiFetch('/toggle_pure_pursuit/', {
      method: 'POST',
      body: JSON.stringify({ enabled })
    })
  },

  togglePathRelaxation(enabled: boolean): Promise<TeleopEnableResponse> {
    return apiFetch('/toggle_path_relaxation/', {
      method: 'POST',
      body: JSON.stringify({ enabled })
    })
  },

  togglePathSmoothing(enabled: boolean): Promise<TeleopEnableResponse> {
    return apiFetch('/toggle_path_smoothing/', {
      method: 'POST',
      body: JSON.stringify({ enabled })
    })
  }
}
