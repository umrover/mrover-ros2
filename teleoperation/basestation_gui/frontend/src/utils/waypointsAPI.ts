import type {
  AutonWaypoint,
  BasicWaypoint,
  WaypointsResponse,
  BasicWaypointsResponse,
  CurrentCourseResponse,
  DeleteWaypointResponse
} from './apiTypes'
import { apiFetch } from './apiFetch'

export const waypointsAPI = {
  getBasic(): Promise<BasicWaypointsResponse> {
    return apiFetch('/waypoints/basic/')
  },

  saveBasic(waypoints: BasicWaypoint[]): Promise<BasicWaypointsResponse> {
    return apiFetch('/waypoints/basic/save/', {
      method: 'POST',
      body: JSON.stringify({ waypoints })
    })
  },

  getAuton(): Promise<WaypointsResponse> {
    return apiFetch('/waypoints/auton/')
  },

  saveAuton(waypoints: AutonWaypoint[]): Promise<WaypointsResponse> {
    return apiFetch('/waypoints/auton/save/', {
      method: 'POST',
      body: JSON.stringify({ waypoints })
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
