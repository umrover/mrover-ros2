# Teleoperation ROS Stack System Summary

## System Status Overview

| System | Status | Consumer | REST API |
|--------|--------|----------|----------|
| Arm | **WEBSOCKET** | arm_consumer.py | - |
| Drive | **WEBSOCKET** | drive_consumer.py | - |
| Science | **HYBRID** | science_consumer.py | science.py |
| Mast | **HYBRID** | mast_consumer.py | mast.py |
| Auton | **REST API** | - | auton.py |
| Nav | **WEBSOCKET** | nav_consumer.py | - |
| Waypoints | **REST API** | - | waypoints.py |

**Note:** For HYBRID systems - WS: datastream only, everything else on REST API

---

## Arm System
**Status: WEBSOCKET**

### Published Topics
| Topic | Type | Declaration |
|-------|------|-------------|
| `arm_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles |
| `ee_pos_cmd` | `mrover/IK` | geometry_msgs/PoseStamped target |
| `ee_vel_cmd` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) |
| `/controller_cmd_vel` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) |
| `sa_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles |

### Subscribed Topics
| Topic | Type | Declaration |
|-------|------|-------------|
| `/arm_controller_state` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit |
| `/sa_controller_state` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit |
| `/arm_joint_data` | `sensor_msgs/JointState` | string[] name, float64[] position, float64[] velocity, float64[] effort |
| `/arm_ik` | `mrover/IK` | geometry_msgs/PoseStamped target |

---

## Drive System
**Status: WEBSOCKET**

### Published Topics
| Topic | Type | Declaration |
|-------|------|-------------|
| `/joystick_cmd_vel` | `geometry_msgs/Twist` | Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z) |

### Subscribed Topics
| Topic | Type | Declaration |
|-------|------|-------------|
| `/drive_left_controller_data` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit |
| `/drive_right_controller_data` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit |
| `/drive_controller_data` | `mrover/ControllerState` | string[] name, string[] state, string[] error, uint8[] limit_hit |

---

## Science System
**Status: HYBRID**

### Subscribed Topics
| Topic | Type | Declaration |
|-------|------|-------------|
| `/led` | `mrover/LED` | bool red, bool green, bool blue, bool is_blinking |
| `/science_oxygen_data` | `mrover/Oxygen` | float64 percent, float64 variance |
| `/science_uv_data` | `mrover/UV` | float64 uv_index, float64 variance |
| `/science_temperature_data` | `sensor_msgs/Temperature` | float64 temperature, float64 variance |
| `/science_humidity_data` | `sensor_msgs/RelativeHumidity` | float64 relative_humidity, float64 variance |
| `/sa_gear_diff_position` | `std_msgs/Float32` | float32 data |

### Services (REST API)
| Service | Type | Request | Response |
|---------|------|---------|----------|
| `/sa_gear_diff_set_position` | `mrover/ServoSetPos` | float32 position, bool is_counterclockwise | bool success |
| `/science_change_heater_auto_shutoff_state` | `mrover/EnableBool` | bool enable | bool success, string message |
| `/sa_enable_limit_switch_sensor_actuator` | `mrover/EnableBool` | bool enable | bool success, string message |

---

## Mast System
**Status: HYBRID**

### Published Topics
| Topic | Type | Declaration |
|-------|------|-------------|
| `/mast_gimbal_throttle_cmd` | `mrover/Throttle` | string[] names, float32[] throttles |

### Services (REST API)
| Service | Type | Request | Response |
|---------|------|---------|----------|
| `/panorama/start` | `mrover/PanoramaStart` | - | - |
| `/panorama/end` | `mrover/PanoramaEnd` | - | bool success, sensor_msgs/Image img |

---

## Auton System
**Status: REST API**

### Services (REST API)
| Service | Type | Request | Response |
|---------|------|---------|----------|
| `/enable_auton` | `mrover/EnableAuton` | bool enable, GPSWaypoint[] waypoints | bool success |
| `/enable_teleop` | `std_srvs/SetBool` | bool data | bool success, string message |

---

## Nav System
**Status: WEBSOCKET**

### Subscribed Topics
| Topic | Type | Declaration |
|-------|------|-------------|
| `/nav_state` | `mrover/StateMachineStateUpdate` | string state_machine_name, string state |
| `/gps/fix` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance |
| `basestation/position` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance |
| `/drone_odometry` | `sensor_msgs/NavSatFix` | float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance |

---

## Waypoints System
**Status: REST API**

---

**Last Updated:** Auto-generated