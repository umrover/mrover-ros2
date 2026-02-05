// src/types/navigation.ts

/**
 * Represents coordinates in Degrees, Minutes, Seconds format.
 */
export interface DMS {
  d: number; // Degrees
  m: number; // Minutes
  s: number; // Seconds
}

/**
 * Represents a full odometry object formatted as DMS.
 */
export interface FormattedOdom {
  lat: DMS;
  lon: DMS;
}

/**
 * Represents a basic odometry object in decimal degrees.
 * Bearing is optional as not all odometry sources provide it.
 */
export interface Odom {
  latitude_deg: number;
  longitude_deg: number;
  bearing_deg?: number;
}

// --- WebSocket Message Payloads ---

/**
 * Standard representation of a quaternion.
 */
export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

/**
 * Message payload for the 'gps_fix' type.
 */
export interface GpsFixMessage {
  type: 'gps_fix';
  latitude: number;
  longitude: number;
  altitude: number;
  status: boolean;
}

/**
 * Message payload for the 'basestation_position' type.
 */
export interface BasestationPositionMessage {
  type: 'basestation_position';
  latitude: number;
  longitude: number;
  status: boolean;
}

/**
 * Message payload for the 'drone_waypoint' type.
 */
export interface DroneWaypointMessage {
  type: 'drone_waypoint';
  latitude: number;
  longitude: number;
  status: boolean;
}

/**
 * Message payload for the 'orientation' type.
 */
export interface OrientationMessage {
  type: 'orientation';
  orientation: Quaternion;
}

/**
 * A union type representing any possible message from the 'nav' channel.
 * This allows TypeScript to correctly narrow down the message type in your watcher.
 */
export type NavMessage =
  | GpsFixMessage
  | BasestationPositionMessage
  | DroneWaypointMessage
  | OrientationMessage;

/**
 * Defines the shape of the component's internal data state for full type safety.
 */
export interface OdomData {
  rover_latitude_deg: number;
  rover_longitude_deg: number;
  rover_bearing_deg: number;
  rover_altitude: number;
  rover_status: boolean;
  drone_latitude_deg: number;
  drone_longitude_deg: number;
  drone_status: boolean;
  basestation_latitude_deg: number;
  basestation_longitude_deg: number;
  basestation_status: boolean;
}