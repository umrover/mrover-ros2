// src/store/types.ts

/** Represents a geographic coordinate. */
export interface LatLng {
  lat: number;
  lon: number;
}

/** Represents a basic waypoint. Define its properties as needed. */
export interface Waypoint {
  name: string;
  lat: number;
  lon: number;
  // Add other waypoint properties if they exist
}

/**
 * State for the 'autonomy' module.
 */
export interface AutonomyState {
  route: Waypoint[];
  waypointList: Waypoint[];
  highlightedWaypoint: number;
  autonEnabled: boolean;
  teleopEnabled: boolean;
  odomFormat: string;
  clickPoint: LatLng;
}

/**
 * State for the 'cameras' module.
 */
export interface CamerasState {
  camsEnabled: boolean[];
  names: string[];
  cameraIdx: number;
  cameraName: string;
  capacity: number;
  qualities: number[];
  streamOrder: number[];
}

/**
 * State for the 'erd' module.
 */
export interface ErdState {
  waypointList: Waypoint[];
  highlightedWaypoint: number;
  searchWaypoint: number;
  odomFormat: string;
  clickPoint: LatLng;
}

/**
 * State for the 'map' module.
 */
export interface MapState {
  odomFormat: string;
}

/**
 * State for the 'websocket' module.
 * It uses dictionary types to store data for multiple WebSocket connections.
 */
export interface WebSocketState {
  messages: { [id: string]: object };
  connectionStatus: { [id:string]: 'connected' | 'disconnected' };
  lastIncomingActivity: { [id: string]: number };
  lastOutgoingActivity: { [id: string]: number };
  flashIn: { [id: string]: boolean };
  flashOut: { [id: string]: boolean };
}

/**
 * The root state of your entire Vuex store.
 * The property names MUST match the module names in `createStore`.
 */
export interface State {
  autonomy: AutonomyState;
  cameras: CamerasState;
  erd: ErdState;
  map: MapState;
  websocket: WebSocketState;
}