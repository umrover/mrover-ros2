export interface GeneralWebSocketMessage {
  type: string;
  [key: string]: unknown; // Allows any other property with an 'unknown' type value
}

export interface WebSocketState {
  messages: { [id: string]: GeneralWebSocketMessage | undefined };
}

// Crucial: Define RootState to encompass your modules
export interface RootState {
  websocket: WebSocketState;
  // Add other module states if you have them, even if they're just 'any' for now
  // For example:
  // map: any;
  // user: any;
  [key: string]: unknown; // Allows other modules to exist without explicit typing
}

export interface ControllerStateMessage {
  type: 'arm_state' | 'sp_state' | 'drive_left_state' | 'drive_right_state' | 'gimbal_state' | 'sp_controller_state' | 'gimbal_controller_state';
  header?: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  names: string[];
  states: string[];
  errors: string[];
  positions: number[];
  velocities: number[];
  currents: number[];
  limits_hit: boolean[];
}

export interface JointStateMessage {
  type: 'drive_left_joint_state' | 'drive_right_joint_state' | 'gimbal_joint_state';
  names: string[];
  positions: number[];
  velocities: number[];
  effort: number[];
}

export interface OrientationMessage {
  type: 'orientation';
  orientation: number[];
}

export interface CalibrationMessage {
  type: 'calibration';
  magnetometer_calibration: number;
  gyroscope_calibration: number;
  acceleration_calibration: number;
}

export interface CmdVelMessage {
  type: 'cmd_vel';
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

export interface OxygenMessage {
  type: 'oxygen';
  percent: number;
}

export interface UVMessage {
  type: 'uv';
  uv_index: number;
}

export interface TemperatureMessage {
  type: 'temperature';
  temperature: number;
}

export interface HumidityMessage {
  type: 'humidity';
  relative_humidity: number;
}

export interface SPHumidityMessage {
  type: 'sp_humidity';
  relative_humidity: number;
}

export interface SPTemperatureMessage {
  type: 'sp_temp';
  temperature: number;
}

export interface SPOxygenMessage {
  type: 'sp_oxygen';
  percent: number;
}

export interface SPUVMessage {
  type: 'sp_uv';
  uv_index: number;
}

export interface SPOzoneMessage {
  type: 'sp_ozone';
  ppb: number;
}

export interface SPCO2Message {
  type: 'sp_co2';
  ppm: number;
}

export interface SPPressureMessage {
  type: 'sp_pressure';
  pressure: number;
}

export type ScienceMessage = OxygenMessage | UVMessage | TemperatureMessage | HumidityMessage | SPHumidityMessage | SPTemperatureMessage | SPOxygenMessage | SPUVMessage | SPOzoneMessage | SPCO2Message | SPPressureMessage;
