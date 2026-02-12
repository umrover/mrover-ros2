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

  togglePathInterpolation(enabled: boolean): Promise<TeleopEnableResponse> {
    return apiFetch('/toggle_path_interpolation/', {
      method: 'POST',
      body: JSON.stringify({ enabled })
    })
  },

  // TODO(object): Implement toggleStereoDetector.
  // It should POST to '/toggle_stereo_detector/' with { enabled }.
  // Follow the exact same pattern as enableTeleop() above.
  toggleStereoDetector(enabled: boolean): Promise<TeleopEnableResponse> {
    // TODO(object): Replace this stub with a real apiFetch call
    return Promise.resolve({ status: 'success' } as TeleopEnableResponse)
  },

  // TODO(object): Same pattern, POST to '/toggle_image_detector/'
  toggleImageDetector(enabled: boolean): Promise<TeleopEnableResponse> {
    // TODO(object): Replace this stub with a real apiFetch call
    return Promise.resolve({ status: 'success' } as TeleopEnableResponse)
  }
}
