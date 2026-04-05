import type {
  AutonWaypoint,
  BasicWaypointRecord,
  WaypointsResponse,
  BasicWaypointsResponse,
  CurrentCourseResponse,
  DeleteWaypointResponse,
  CreateWaypointResponse,
  CreateBasicWaypointResponse
} from './apiTypes'
import { apiFetch } from './apiFetch'

export const waypointsAPI = {
  getBasic(): Promise<BasicWaypointsResponse> {
    return apiFetch('/waypoints/basic/')
  },

  createBasic(waypoint: Omit<BasicWaypointRecord, 'db_id'>): Promise<CreateBasicWaypointResponse> {
    return apiFetch('/waypoints/basic/', {
      method: 'POST',
      body: JSON.stringify(waypoint)
    })
  },

  updateBasic(id: number, fields: Partial<BasicWaypointRecord>): Promise<CreateBasicWaypointResponse> {
    return apiFetch(`/waypoints/basic/${id}/`, {
      method: 'PATCH',
      body: JSON.stringify(fields)
    })
  },

  deleteBasicWaypoint(id: number): Promise<DeleteWaypointResponse> {
    return apiFetch(`/waypoints/basic/${id}/`, {
      method: 'DELETE'
    })
  },

  getAuton(): Promise<WaypointsResponse> {
    return apiFetch('/waypoints/auton/')
  },

  createAuton(waypoint: Omit<AutonWaypoint, 'db_id' | 'deletable'>): Promise<CreateWaypointResponse> {
    return apiFetch('/waypoints/auton/', {
      method: 'POST',
      body: JSON.stringify(waypoint)
    })
  },

  updateAuton(id: number, fields: Partial<AutonWaypoint>): Promise<CreateWaypointResponse> {
    return apiFetch(`/waypoints/auton/${id}/`, {
      method: 'PATCH',
      body: JSON.stringify(fields)
    })
  },

  getCurrentAutonCourse(): Promise<CurrentCourseResponse> {
    return apiFetch('/waypoints/auton/current/')
  },

  saveCurrentAutonCourse(course: AutonWaypoint[]): Promise<CurrentCourseResponse> {
    return apiFetch('/waypoints/auton/current/save/', {
      method: 'POST',
      body: JSON.stringify({ waypoints: course })
    })
  },

  deleteAutonWaypoint(waypoint: AutonWaypoint): Promise<DeleteWaypointResponse> {
    return apiFetch(`/waypoints/auton/${waypoint.db_id}/`, {
      method: 'DELETE',
      body: JSON.stringify(waypoint)
    })
  },

  deleteAll(): Promise<BasicWaypointsResponse> {
    return apiFetch('/waypoints/basic/clear/', { method: 'DELETE' })
  },

  clearAuton(): Promise<DeleteWaypointResponse> {
    return apiFetch('/waypoints/auton/clear/', { method: 'DELETE' })
  },

  clearAllAuton(): Promise<DeleteWaypointResponse> {
    return apiFetch('/waypoints/auton/clear/all/', { method: 'DELETE' })
  }
}
