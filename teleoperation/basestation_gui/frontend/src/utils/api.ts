/**
 * REST API utilities for teleoperation system
 */

import type {
  AutonWaypoint,
  BasicWaypoint,
  WaypointsResponse,
  BasicWaypointsResponse,
  CurrentCourseResponse,
  DeleteWaypointResponse,
  AutonEnableResponse,
  TeleopEnableResponse,
  AutonEnableWaypoint,
  APIResponse
} from './types'

const API_BASE = '/api'

// Waypoints API
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
      body: JSON.stringify({ course })
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
  }
}

// Auton API
export const autonAPI = {
  async enable(enabled: boolean, waypoints: AutonEnableWaypoint[]): Promise<AutonEnableResponse> {
    const response = await fetch(`${API_BASE}/auton/enable/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enabled, waypoints })
    })

    if (!response.ok) {
      return { status: 'error', message: `HTTP ${response.status}: ${response.statusText}` }
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
      return { status: 'error', message: `HTTP ${response.status}: ${response.statusText}` }
    }

    return response.json()
  }
}

// Science API
export const scienceAPI = {
  async setHeater(heaterName: string, enable: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/heater/${heaterName}/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable })
    })
    return response.json()
  },

  async setGearDiffPosition(position: number, isCounterclockwise: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/gear-diff/position/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ position, is_counterclockwise: isCounterclockwise })
    })
    return response.json()
  },

  async setAutoShutoff(enable: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/auto-shutoff/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable })
    })
    return response.json()
  },

  async setWhiteLEDs(site: string, enable: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/white-leds/${site}/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable })
    })
    return response.json()
  },

  async setLimitSwitch(enable: boolean): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/science/limit-switch/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable })
    })
    return response.json()
  }
}

// Mast API
export const mastAPI = {
  async startPanorama(): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/mast/panorama/start/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  },

  async stopPanorama(): Promise<APIResponse> {
    const response = await fetch(`${API_BASE}/mast/panorama/stop/`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    })
    return response.json()
  }
}
