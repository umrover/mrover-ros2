import type {
  AutonWaypoint,
  BasicWaypoint,
  WaypointsResponse,
  BasicWaypointsResponse,
  CurrentCourseResponse,
  DeleteWaypointResponse
} from './apiTypes'

const API_BASE = '/api'

export const waypointsAPI = {
  async getBasic(): Promise<BasicWaypointsResponse> {
    const response = await fetch(`${API_BASE}/waypoints/basic/`)
    return response.json()
  },

  async saveBasic(waypoints: BasicWaypoint[]): Promise<BasicWaypointsResponse> {
    const response = await fetch(`${API_BASE}/waypoints/basic/save/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ waypoints })
    })
    return response.json()
  },

  async getAuton(): Promise<WaypointsResponse> {
    const response = await fetch(`${API_BASE}/waypoints/auton/`)
    return response.json()
  },

  async saveAuton(waypoints: AutonWaypoint[]): Promise<WaypointsResponse> {
    const response = await fetch(`${API_BASE}/waypoints/auton/save/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ waypoints })
    })
    return response.json()
  },

  async getCurrentAutonCourse(): Promise<CurrentCourseResponse> {
    const response = await fetch(`${API_BASE}/waypoints/auton/current/`)
    return response.json()
  },

  async saveCurrentAutonCourse(course: AutonWaypoint[]): Promise<CurrentCourseResponse> {
    const response = await fetch(`${API_BASE}/waypoints/auton/current/save/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ waypoints: course })
    })
    return response.json()
  },

  async deleteAutonWaypoint(waypoint: AutonWaypoint): Promise<DeleteWaypointResponse> {
    const response = await fetch(`${API_BASE}/waypoints/auton/${waypoint.id}/`, {
      method: 'DELETE',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(waypoint)
    })
    return response.json()
  },

  async deleteAll(): Promise<BasicWaypointsResponse> {
    const response = await fetch(`${API_BASE}/waypoints/basic/clear/`, {
      method: 'DELETE',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  }
}
