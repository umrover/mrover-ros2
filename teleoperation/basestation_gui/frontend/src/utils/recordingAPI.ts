import type {
  APIResponse,
  RecordingCreateResponse,
  RecordingsListResponse,
  RecordingWaypointsResponse
} from './apiTypes'

const API_BASE = '/api'

export const recordingAPI = {
  async create(name: string, isDrone: boolean): Promise<RecordingCreateResponse> {
    const response = await fetch(`${API_BASE}/waypoints/recordings/create/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name, is_drone: isDrone })
    })
    return response.json()
  },

  async stop(): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/waypoints/recordings/stop/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  },

  async addWaypoint(recordingId: number, lat: number, lon: number, sequence: number): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/waypoints/recordings/${recordingId}/waypoints/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ lat, lon, sequence })
    })
    return response.json()
  },

  async getAll(): Promise<RecordingsListResponse> {
    const response = await fetch(`${API_BASE}/waypoints/recordings/`)
    return response.json()
  },

  async getWaypoints(recordingId: number): Promise<RecordingWaypointsResponse> {
    const response = await fetch(`${API_BASE}/waypoints/recordings/${recordingId}/waypoints/`)
    return response.json()
  },

  async delete(recordingId: number): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/waypoints/recordings/${recordingId}/`, {
      method: 'DELETE',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  },

  async deleteAll(): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/waypoints/recordings/clear/`, {
      method: 'DELETE',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  }
}
