# Teleoperation Views - Topics and Services Reference

**Note:** This document only includes topics and services that are actively used by the frontend. Topics subscribed to by consumers but not consumed by any frontend component are excluded.

---

## View: AutonTask

**Path:** `/AutonTask`
**WebSockets:** `drive`, `nav`, `science`, `mast`

### Published Topics
| Topic | Type | Declaration | Component |
|-------|------|-------------|-----------|
| `/joystick_cmd_vel` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) | DriveControls |
| `/mast_gimbal_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles | MastGimbalControls |

### Subscribed Topics
| Topic | Type | Declaration | Component |
|-------|------|-------------|-----------|
| `/drive_left_controller_data` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit | ControllerDataTable |
| `/drive_right_controller_data` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit | ControllerDataTable |
| `/nav_state` | `mrover/StateMachineStateUpdate` | string state_machine_name, string state | AutonTask |
| `/gps/fix` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | AutonRoverMap |
| `basestation/position` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | AutonRoverMap |
| `/drone_odometry` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | AutonRoverMap |
| `/led` | `mrover/LED` | bool red, bool green, bool blue, bool is_blinking | AutonTask |

### Computed Data
| Data | Source | Component |
|------|--------|-----------|
| Orientation (Quaternion) | TF transform map→base_link | AutonRoverMap |

### Services (REST API)
| Service | Type | Request | Response | Component |
|---------|------|---------|----------|-----------|
| `/enable_auton` | `mrover/EnableAuton` | bool enable, GPSWaypoint[] waypoints | bool success | AutonWaypointEditor |
| `/enable_teleop` | `std_srvs/SetBool` | bool data | bool success, string message | AutonWaypointEditor |
| Waypoints CRUD | - | - | - | AutonWaypointEditor |

---

## View: DMTask (Delivery Mission)

**Path:** `/DMTask`
**WebSockets:** `arm`, `drive`, `mast`, `nav`

### Published Topics
| Topic | Type | Declaration | Component |
|-------|------|-------------|-----------|
| `arm_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles | ArmControls |
| `ee_pos_cmd` | `mrover/IK` | geometry_msgs/PoseStamped target | ArmControls |
| `ee_vel_cmd` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) | ArmControls |
| `/controller_cmd_vel` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) | ArmControls |
| `sa_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles | ArmControls |
| `/joystick_cmd_vel` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) | DriveControls |
| `/mast_gimbal_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles | MastGimbalControls |

### Subscribed Topics
| Topic | Type | Declaration | Component |
|-------|------|-------------|-----------|
| `/arm_controller_state` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit | ControllerDataTable |
| `/arm_joint_data` | `sensor_msgs/JointState` | string[] name, float64[] position, float64[] velocity, float64[] effort | Rover3D |
| `/arm_ik` | `mrover/IK` | geometry_msgs/PoseStamped target | Rover3D |
| `/drive_controller_data` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit | ControllerDataTable |
| `/gps/fix` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | OdometryReading, BasicRoverMap |
| `basestation/position` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | OdometryReading, BasicRoverMap |
| `/drone_odometry` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | OdometryReading, BasicRoverMap |

### Computed Data
| Data | Source | Component |
|------|--------|-----------|
| Orientation (Quaternion) | TF transform map→base_link | OdometryReading, BasicRoverMap |

### Services (REST API)
| Service | Type | Request | Response | Component |
|---------|------|---------|----------|-----------|
| Waypoints CRUD | - | - | - | BasicWaypointEditor |

---

## View: ESTask (Equipment Servicing)

**Path:** `/ESTask`
**WebSockets:** `arm`, `drive`

### Published Topics
| Topic | Type | Declaration | Component |
|-------|------|-------------|-----------|
| `arm_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles | ArmControls |
| `ee_pos_cmd` | `mrover/IK` | geometry_msgs/PoseStamped target | ArmControls |
| `ee_vel_cmd` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) | ArmControls |
| `/controller_cmd_vel` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) | ArmControls |
| `sa_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles | ArmControls |

### Subscribed Topics
| Topic | Type | Declaration | Component |
|-------|------|-------------|-----------|
| `/arm_controller_state` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit | ControllerDataTable |
| `/arm_joint_data` | `sensor_msgs/JointState` | string[] name, float64[] position, float64[] velocity, float64[] effort | Rover3D |
| `/arm_ik` | `mrover/IK` | geometry_msgs/PoseStamped target | Rover3D |
| `/drive_controller_data` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit | ControllerDataTable |

---

## View: SPTask (Science Payload)

**Path:** `/SPTask`
**WebSockets:** `arm`, `mast`, `nav`, `science`

### Published Topics
| Topic | Type | Declaration | Component |
|-------|------|-------------|-----------|
| `/joystick_cmd_vel` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) | DriveControls |
| `/mast_gimbal_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles | MastGimbalControls |

### Subscribed Topics
| Topic | Type | Declaration | Component |
|-------|------|-------------|-----------|
| `/sa_controller_state` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit | ControllerDataTable |
| `/drive_controller_data` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit | ControllerDataTable |
| `/gps/fix` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | OdometryReading, BasicRoverMap |
| `basestation/position` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | OdometryReading, BasicRoverMap |
| `/drone_odometry` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance | OdometryReading, BasicRoverMap |
| `/science_oxygen_data` | `mrover/Oxygen` | float64 percent, float64 variance | SensorData |
| `/science_uv_data` | `mrover/UV` | float64 uv_index, float64 variance | SensorData |
| `/science_temperature_data` | `sensor_msgs/Temperature` | float64 temperature, float64 variance | SensorData |
| `/science_humidity_data` | `sensor_msgs/RelativeHumidity` | float64 relative_humidity, float64 variance | SensorData |

### Computed Data
| Data | Source | Component |
|------|--------|-----------|
| Orientation (Quaternion) | TF transform map→base_link | OdometryReading, BasicRoverMap |

### Services (REST API)
| Service | Type | Request | Response | Component |
|---------|------|---------|----------|-----------|
| `/sa_gear_diff_set_position` | `mrover/ServoSetPos` | float32 position, bool is_counterclockwise | bool success | HexHub |
| `/panorama/start` | `mrover/PanoramaStart` | - | - | PanoCam |
| `/panorama/end` | `mrover/PanoramaEnd` | - | bool success, sensor_msgs/Image img | PanoCam |
| Waypoints CRUD | - | - | - | BasicWaypointEditor |

---

## Summary Statistics

| View | WebSockets | Published Topics | Subscribed Topics | Computed Data | REST Services |
|------|------------|------------------|-------------------|---------------|---------------|
| AutonTask | 4 | 2 | 7 | 1 | 3 |
| DMTask | 4 | 7 | 7 | 1 | 1 |
| ESTask | 2 | 5 | 4 | 0 | 0 |
| SPTask | 4 | 2 | 9 | 1 | 4 |

---

**Last Updated:** Auto-generated
