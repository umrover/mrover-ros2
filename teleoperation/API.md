# Teleoperation API Documentation

Hybrid REST API + WebSocket architecture for the MRover teleoperation system.

## Architecture Overview

**WebSockets** - Real-time bidirectional streaming:
- ROS topic data forwarding (sensor readings, states)
- Latency-critical controller inputs
- Continuous telemetry streams

**REST APIs** - Request/response operations:
- CRUD operations (waypoints, configurations)
- One-time commands (enable/disable, service calls)
- State queries without timing constraints

---

## REST API Endpoints

Base URL: `http://localhost:8000/api/`

### Waypoints (`backend/api/views/waypoints.py`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/waypoints/basic/` | Get list of all basic waypoints |
| POST | `/waypoints/basic/save/` | Save basic waypoints list |
| GET | `/waypoints/auton/` | Get list of all auton waypoints |
| POST | `/waypoints/auton/save/` | Save auton waypoints list |
| GET | `/waypoints/basic/current/` | Get current basic course |
| POST | `/waypoints/basic/current/save/` | Save current basic course |
| GET | `/waypoints/auton/current/` | Get current auton course |
| POST | `/waypoints/auton/current/save/` | Save current auton course |
| DELETE | `/waypoints/auton/<waypoint_id>/` | Delete waypoint from auton course |

### Auton (`backend/api/views/auton.py`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/auton/enable/` | Enable/disable autonomous navigation with waypoints |
| POST | `/teleop/enable/` | Enable/disable teleoperation mode |

### Science (`backend/api/views/science.py`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/science/heater/<heater_name>/` | Enable/disable heater (a0, a1, b0, b1) |
| POST | `/science/gear-diff/position/` | Set gear differential position |
| POST | `/science/auto-shutoff/` | Enable/disable heater auto shutoff |
| POST | `/science/white-leds/<site>/` | Control white LEDs for site (a or b) |
| POST | `/science/limit-switch/` | Enable/disable limit switch sensor actuator |

### Mast (`backend/api/views/mast.py`)

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/mast/panorama/start/` | Start panorama capture |
| POST | `/mast/panorama/stop/` | Stop panorama capture and save image |

---

## WebSocket Consumers

### Arm Consumer
**WebSocket:** `ws://[host]/ws/arm/`

#### Receive (from GUI)
| Type | Description |
|------|-------------|
| `ra_controller` | Robotic arm controller input |
| `ra_mode` | Set RA control mode |
| `sa_controller` | Science arm controller input |
| `sa_mode` | Set SA control mode |

#### Send (to GUI)
| Type | ROS Topic | Description |
|------|-----------|-------------|
| `arm_state` | `/arm_controller_state` | Arm controller state |
| `sa_state` | `/sa_controller_state` | Science arm controller state |
| `fk` | `/arm_joint_data` | Forward kinematics joint data |
| `ik_target` | `/arm_ik` | IK target position |

#### Published ROS Topics
- `/arm_throttle_cmd`, `/ee_pos_cmd`, `/ee_vel_cmd`, `/controller_cmd_vel`, `/sa_throttle_cmd`

---

### Drive Consumer
**WebSocket:** `ws://[host]/ws/drive/`

#### Receive (from GUI)
| Type | Description |
|------|-------------|
| `joystick` | Joystick drive input |

#### Send (to GUI)
| Type | ROS Topic | Description |
|------|-----------|-------------|
| `drive_left_state` | `/drive_left_controller_data` | Left drive controller state |
| `drive_right_state` | `/drive_right_controller_data` | Right drive controller state |
| `drive_state` | `/drive_controller_data` | Drive controller state |

#### Published ROS Topics
- `/joystick_cmd_vel`

---

### Mast Consumer
**WebSocket:** `ws://[host]/ws/mast/`

#### Receive (from GUI)
| Type | Description |
|------|-------------|
| `mast_keyboard` | Mast gimbal keyboard input |

#### Published ROS Topics
- `/mast_gimbal_throttle_cmd`

---

### Nav Consumer
**WebSocket:** `ws://[host]/ws/nav/`

#### Send (to GUI)
| Type | ROS Topic | Description | Frequency |
|------|-----------|-------------|-----------|
| `nav_state` | `/nav_state` | Navigation state machine state | Event-based |
| `gps_fix` | `/gps/fix` | GPS fix data | Event-based |
| `basestation_position` | `basestation/position` | Basestation GPS position | Event-based |
| `drone_waypoint` | `/drone_odometry` | Drone position | Event-based |
| `orientation` | TF: map→base_link | Robot orientation quaternion | 10 Hz |

---

### Science Consumer
**WebSocket:** `ws://[host]/ws/science/`

#### Send (to GUI - Sensor Streaming)
| Type | ROS Topic | Description |
|------|-----------|-------------|
| `led` | `/led` | LED state |
| `thermistors` | `/science_thermistors` | Thermistor readings |
| `heater_states` | `/science_heater_state` | Heater states |
| `oxygen` | `/science_oxygen_data` | Oxygen sensor data |
| `methane` | `/science_methane_data` | Methane sensor data |
| `uv` | `/science_uv_data` | UV sensor data |
| `temperature` | `/science_temperature_data` | Temperature readings |
| `humidity` | `/science_humidity_data` | Humidity readings |
| `hexhub_site` | `/sa_gear_diff_position` | Gear differential position |

---

### Waypoints Consumer (DEPRECATED)
**WebSocket:** `ws://[host]/ws/waypoints/`

All waypoint operations have been migrated to REST API. This consumer can be removed once frontend is updated.

---

## Migration Status

| Consumer | Total Operations | Migrated to REST | WebSocket Only | Status |
|----------|-----------------|------------------|----------------|---------|
| **Waypoints** | 9 | 9 (100%) | 0 | ✅ Complete - can remove consumer |
| **Auton** | 2 | 2 (100%) | 0 | ✅ Complete - simplified to streaming |
| **Science** | 14 | 5 controls | 9 streaming | ✅ Complete - hybrid model |
| **Mast** | 2 | 1 panorama | 1 gimbal control | ✅ Complete - hybrid model |
| **Arm** | 6 | 0 | 6 | ✅ WebSocket only (latency-critical) |
| **Drive** | 4 | 0 | 4 | ✅ WebSocket only (latency-critical) |
| **Nav** | 5 | 0 | 5 | ✅ WebSocket only (streaming) |

### Implementation Status
- ✅ REST API infrastructure setup
- ✅ Django REST Framework configured
- ✅ All REST endpoints implemented
- ✅ URL routing configured
- ✅ Frontend migration complete - all components migrated

---

## Implementation Files

### Backend
- `backend/api/urls.py` - API URL routing
- `backend/api/views/waypoints.py` - Waypoints REST views
- `backend/api/views/auton.py` - Auton REST views
- `backend/api/views/science.py` - Science REST views
- `backend/api/views/mast.py` - Mast REST views

### Frontend
- `frontend/src/utils/api.ts` - Centralized REST API utility with typed methods

### Configuration
- `basestation_gui/settings.py` - Django REST Framework settings
- `basestation_gui/urls.py` - Main URL routing with `/api/` prefix

---

## Notes

- All REST endpoints return JSON with `status` field (`"success"` or `"error"`)
- Error responses include `message` field with error details
- HTTP status codes: 200 (success), 400 (bad request), 404 (not found), 500 (server error)
- Authentication is disabled for development (can be enabled for production)
- CSRF protection is handled by Django middleware
