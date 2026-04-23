import type {
  APIResponse,
  RecordingCreateResponse,
  RecordingsListResponse,
  RecordingWaypointsResponse
} from './apiTypes'
import { apiFetch } from './apiFetch'

export const recordingAPI = {
  create(name: string, isDrone: boolean): Promise<RecordingCreateResponse> {
    return apiFetch('/recordings/create/', {
      method: 'POST',
      body: JSON.stringify({ name, is_drone: isDrone })
    })
  },

  stop(): Promise<APIResponse> {
    return apiFetch('/recordings/stop/', { method: 'POST' })
  },

  addWaypoint(recordingId: number, lat: number, lon: number, sequence: number): Promise<APIResponse> {
    return apiFetch(`/recordings/${recordingId}/waypoints/`, {
      method: 'POST',
      body: JSON.stringify({ lat, lon, sequence })
    })
  },

  getAll(): Promise<RecordingsListResponse> {
    return apiFetch('/recordings/')
  },

  getWaypoints(recordingId: number): Promise<RecordingWaypointsResponse> {
    return apiFetch(`/recordings/${recordingId}/waypoints/`)
  },

  delete(recordingId: number): Promise<APIResponse> {
    return apiFetch(`/recordings/${recordingId}/`, { method: 'DELETE' })
  },

  deleteAll(): Promise<APIResponse> {
    return apiFetch('/recordings/clear/', { method: 'DELETE' })
  }
}
