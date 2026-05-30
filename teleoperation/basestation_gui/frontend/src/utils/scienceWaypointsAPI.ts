import type {
  APIResponse,
  GpsSnapshotResponse,
  ScienceWaypointsResponse,
  CreateScienceWaypointResponse,
} from './apiTypes'
import { apiFetch } from './apiFetch'

export const scienceWaypointsAPI = {
  getGpsSnapshot(): Promise<GpsSnapshotResponse> {
    return apiFetch('/science-waypoints/gps-snapshot/')
  },

  getAll(): Promise<ScienceWaypointsResponse> {
    return apiFetch('/science-waypoints/')
  },

  create(waypoint: { name: string; lat: number; lon: number; altitude: number }): Promise<CreateScienceWaypointResponse> {
    return apiFetch('/science-waypoints/', {
      method: 'POST',
      body: JSON.stringify(waypoint),
    })
  },

  delete(id: number): Promise<APIResponse> {
    return apiFetch(`/science-waypoints/${id}/`, { method: 'DELETE' })
  },

  clear(): Promise<APIResponse> {
    return apiFetch('/science-waypoints/clear/', { method: 'DELETE' })
  },

  reset(): Promise<APIResponse> {
    return apiFetch('/science-waypoints/reset/', { method: 'POST' })
  },
}
