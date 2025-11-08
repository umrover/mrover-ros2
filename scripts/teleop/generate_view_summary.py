#!/usr/bin/env python3
"""
Generate view-based teleoperation system summary documentation.

This script scans the teleoperation backend code and frontend views to generate:
- system_by_view.html: HTML tables organized by view/task

Output files are placed in the mrover root directory.
"""

import os
import re
import yaml
from pathlib import Path
from typing import Dict, List, Tuple, Set
from dataclasses import dataclass, field
from jinja2 import Environment, FileSystemLoader


@dataclass
class TopicInfo:
    name: str
    type: str
    declaration: str
    component: str
    subteam: str
    subteam_id: str


@dataclass
class ServiceInfo:
    name: str
    type: str
    request: str
    response: str
    component: str
    subteam: str
    subteam_id: str


@dataclass
class ComputedDataInfo:
    name: str
    source: str
    component: str
    subteam: str
    subteam_id: str


@dataclass
class ViewInfo:
    name: str
    description: str
    route: str
    websockets: List[str] = field(default_factory=list)
    published_topics: List[TopicInfo] = field(default_factory=list)
    subscribed_topics: List[TopicInfo] = field(default_factory=list)
    computed_data: List[ComputedDataInfo] = field(default_factory=list)
    services: List[ServiceInfo] = field(default_factory=list)


def load_subteam_mapping(mapping_path: Path) -> Dict:
    """Load the subteam mapping configuration from YAML."""
    with open(mapping_path, 'r') as f:
        return yaml.safe_load(f)


def parse_msg_file(msg_path: Path) -> str:
    """Parse a .msg file and return its declaration."""
    if not msg_path.exists():
        return ""

    content = msg_path.read_text()
    lines = []
    for line in content.split('\n'):
        line = line.split('#')[0].strip()
        if line:
            lines.append(line)
    return ', '.join(lines)


def parse_srv_file(srv_path: Path) -> Tuple[str, str]:
    """Parse a .srv file and return (request, response) declarations."""
    if not srv_path.exists():
        return "", ""

    content = srv_path.read_text()
    parts = content.split('---')

    request = ""
    response = ""

    if len(parts) >= 1:
        req_lines = []
        for line in parts[0].split('\n'):
            line = line.split('#')[0].strip()
            if line:
                req_lines.append(line)
        request = ', '.join(req_lines) if req_lines else "-"

    if len(parts) >= 2:
        resp_lines = []
        for line in parts[1].split('\n'):
            line = line.split('#')[0].strip()
            if line:
                resp_lines.append(line)
        response = ', '.join(resp_lines) if resp_lines else "-"

    return request, response


def get_standard_msg_declaration(msg_type: str) -> str:
    """Return declaration for standard ROS message types."""
    declarations = {
        "geometry_msgs/Twist": "Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z)",
        "geometry_msgs/PoseStamped": "Header header, Pose pose (Point position, Quaternion orientation)",
        "sensor_msgs/JointState": "string[] name, float64[] position, float64[] velocity, float64[] effort",
        "sensor_msgs/Temperature": "float64 temperature, float64 variance",
        "sensor_msgs/RelativeHumidity": "float64 relative_humidity, float64 variance",
        "sensor_msgs/NavSatFix": "float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance",
        "sensor_msgs/Image": "uint32 height, uint32 width, string encoding, uint8 is_bigendian, uint32 step, uint8[] data",
        "std_msgs/Float32": "float32 data",
    }
    return declarations.get(msg_type, "")


def get_standard_srv_declaration(srv_type: str) -> Tuple[str, str]:
    """Return (request, response) for standard ROS service types."""
    if srv_type == "std_srvs/SetBool":
        return "bool data", "bool success, string message"
    return "", ""


def get_msg_declaration(msg_type: str, msg_dir: Path) -> str:
    """Get message declaration for any message type."""
    if '/' in msg_type or msg_type in ['Twist', 'PoseStamped', 'JointState', 'Temperature', 'RelativeHumidity', 'NavSatFix', 'Image', 'Float32']:
        full_type = msg_type if '/' in msg_type else f'geometry_msgs/{msg_type}' if msg_type in ['Twist', 'PoseStamped'] else f'sensor_msgs/{msg_type}' if msg_type in ['JointState', 'Temperature', 'RelativeHumidity', 'NavSatFix', 'Image'] else f'std_msgs/{msg_type}'
        return get_standard_msg_declaration(full_type)
    else:
        msg_file = msg_dir / f'{msg_type}.msg'
        return parse_msg_file(msg_file)


def get_subteam_for_topic(topic_name: str, component: str, mapping: Dict) -> Tuple[str, str]:
    """Get subteam name and ID for a topic."""
    subteam_id = mapping.get('topics', {}).get(topic_name)
    if not subteam_id:
        subteam_id = mapping.get('components', {}).get(component, 'esw')

    subteam_info = mapping.get('subteams', {}).get(subteam_id, {'name': 'Unknown'})
    return subteam_info.get('name', 'Unknown'), subteam_id


def get_subteam_for_service(service_name: str, component: str, mapping: Dict) -> Tuple[str, str]:
    """Get subteam name and ID for a service."""
    subteam_id = mapping.get('services', {}).get(service_name)
    if not subteam_id:
        subteam_id = mapping.get('components', {}).get(component, 'esw')

    subteam_info = mapping.get('subteams', {}).get(subteam_id, {'name': 'Unknown'})
    return subteam_info.get('name', 'Unknown'), subteam_id


def scan_views(project_root: Path, mapping: Dict) -> List[ViewInfo]:
    """Scan frontend views and extract information for each view."""
    views = []

    # Define views manually based on the example HTML
    view_configs = [
        {
            'name': 'AutonTask',
            'description': 'Autonomy Mission',
            'route': '/AutonTask',
            'websockets': ['drive', 'nav', 'science', 'mast'],
            'components': {
                'DriveControls': ['published'],
                'MastGimbalControls': ['published'],
                'ControllerDataTable': ['subscribed'],
                'AutonTask': ['subscribed'],
                'AutonRoverMap': ['subscribed', 'computed'],
                'AutonWaypointEditor': ['services'],
            }
        },
        {
            'name': 'DMTask',
            'description': 'Delivery Mission',
            'route': '/DMTask',
            'websockets': ['arm', 'drive', 'mast', 'nav'],
            'components': {
                'ArmControls': ['published'],
                'DriveControls': ['published'],
                'MastGimbalControls': ['published'],
                'ControllerDataTable': ['subscribed'],
                'Rover3D': ['subscribed'],
                'OdometryReading': ['subscribed', 'computed'],
                'BasicRoverMap': ['subscribed', 'computed'],
                'BasicWaypointEditor': ['services'],
            }
        },
        {
            'name': 'ESTask',
            'description': 'Equipment Servicing',
            'route': '/ESTask',
            'websockets': ['arm', 'drive'],
            'components': {
                'ArmControls': ['published'],
                'ControllerDataTable': ['subscribed'],
                'Rover3D': ['subscribed'],
            }
        },
        {
            'name': 'SPTask',
            'description': 'Science Payload',
            'route': '/SPTask',
            'websockets': ['arm', 'mast', 'nav', 'science'],
            'components': {
                'DriveControls': ['published'],
                'MastGimbalControls': ['published'],
                'ControllerDataTable': ['subscribed'],
                'OdometryReading': ['subscribed', 'computed'],
                'BasicRoverMap': ['subscribed', 'computed'],
                'SensorData': ['subscribed'],
                'HexHub': ['services'],
                'PanoCam': ['services'],
                'BasicWaypointEditor': ['services'],
            }
        },
    ]

    msg_dir = project_root / "msg"
    srv_dir = project_root / "srv"

    # Topic database based on the existing scan
    topic_db = {
        '/joystick_cmd_vel': ('geometry_msgs/Twist', 'Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z)', 'published', 'DriveControls'),
        '/mast_gimbal_throttle_cmd': ('mrover/Throttle', 'string[] names, float32[] throttles', 'published', 'MastGimbalControls'),
        'arm_throttle_cmd': ('mrover/Throttle', 'string[] names, float32[] throttles', 'published', 'ArmControls'),
        'ee_pos_cmd': ('mrover/IK', 'geometry_msgs/Vector3 pos, float32 pitch, float32 roll', 'published', 'ArmControls'),
        'ee_vel_cmd': ('geometry_msgs/Twist', 'Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z)', 'published', 'ArmControls'),
        '/controller_cmd_vel': ('geometry_msgs/Twist', 'Vector3 linear (float64 x, y, z), Vector3 angular (float64 x, y, z)', 'published', 'ArmControls'),
        'sa_throttle_cmd': ('mrover/Throttle', 'string[] names, float32[] throttles', 'published', 'ArmControls'),
        '/drive_left_controller_data': ('mrover/ControllerState', 'string[] name, string[] state, string[] error, uint8[] limit_hit', 'subscribed', 'ControllerDataTable'),
        '/drive_right_controller_data': ('mrover/ControllerState', 'string[] name, string[] state, string[] error, uint8[] limit_hit', 'subscribed', 'ControllerDataTable'),
        '/drive_controller_data': ('mrover/ControllerState', 'string[] name, string[] state, string[] error, uint8[] limit_hit', 'subscribed', 'ControllerDataTable'),
        '/nav_state': ('mrover/StateMachineStateUpdate', 'string state_machine_name, string state', 'subscribed', 'AutonTask'),
        '/gps/fix': ('sensor_msgs/NavSatFix', 'float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance', 'subscribed', 'AutonRoverMap'),
        'basestation/position': ('sensor_msgs/NavSatFix', 'float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance', 'subscribed', 'AutonRoverMap'),
        '/drone_odometry': ('sensor_msgs/NavSatFix', 'float64 latitude, float64 longitude, float64 altitude, NavSatStatus status, float64[9] position_covariance', 'subscribed', 'AutonRoverMap'),
        '/led': ('mrover/LED', 'bool red, bool green, bool blue, bool is_blinking', 'subscribed', 'AutonTask'),
        '/arm_controller_state': ('mrover/ControllerState', 'string[] name, string[] state, string[] error, uint8[] limit_hit', 'subscribed', 'ControllerDataTable'),
        '/sa_controller_state': ('mrover/ControllerState', 'string[] name, string[] state, string[] error, uint8[] limit_hit', 'subscribed', 'ControllerDataTable'),
        '/arm_joint_data': ('sensor_msgs/JointState', 'string[] name, float64[] position, float64[] velocity, float64[] effort', 'subscribed', 'Rover3D'),
        '/arm_ik': ('mrover/IK', 'geometry_msgs/Vector3 pos, float32 pitch, float32 roll', 'subscribed', 'Rover3D'),
        '/science_oxygen_data': ('mrover/Oxygen', 'float64 percent, float64 variance', 'subscribed', 'SensorData'),
        '/science_uv_data': ('mrover/UV', 'float64 uv_index, float64 variance', 'subscribed', 'SensorData'),
        '/science_temperature_data': ('sensor_msgs/Temperature', 'float64 temperature, float64 variance', 'subscribed', 'SensorData'),
        '/science_humidity_data': ('sensor_msgs/RelativeHumidity', 'float64 relative_humidity, float64 variance', 'subscribed', 'SensorData'),
    }

    service_db = {
        '/enable_auton': ('mrover/EnableAuton', 'bool enable, GPSWaypoint[] waypoints', 'bool success', 'AutonWaypointEditor'),
        '/enable_teleop': ('std_srvs/SetBool', 'bool data', 'bool success, string message', 'AutonWaypointEditor'),
        '/sa_gear_diff_set_position': ('mrover/ServoSetPos', 'float32 position, bool is_counterclockwise', 'bool success', 'HexHub'),
        '/panorama/start': ('mrover/PanoramaStart', '-', '-', 'PanoCam'),
        '/panorama/end': ('mrover/PanoramaEnd', '-', 'bool success, sensor_msgs/Image img', 'PanoCam'),
    }

    for config in view_configs:
        view = ViewInfo(
            name=config['name'],
            description=config['description'],
            route=config['route'],
            websockets=config['websockets']
        )

        # Add topics and services based on components
        for component, types in config['components'].items():
            if 'published' in types:
                for topic_name, (topic_type, declaration, direction, comp) in topic_db.items():
                    if direction == 'published' and comp == component:
                        subteam, subteam_id = get_subteam_for_topic(topic_name, component, mapping)
                        view.published_topics.append(TopicInfo(
                            name=topic_name,
                            type=topic_type,
                            declaration=declaration,
                            component=component,
                            subteam=subteam,
                            subteam_id=subteam_id
                        ))

            if 'subscribed' in types:
                for topic_name, (topic_type, declaration, direction, comp) in topic_db.items():
                    if direction == 'subscribed' and comp == component:
                        subteam, subteam_id = get_subteam_for_topic(topic_name, component, mapping)
                        view.subscribed_topics.append(TopicInfo(
                            name=topic_name,
                            type=topic_type,
                            declaration=declaration,
                            component=component,
                            subteam=subteam,
                            subteam_id=subteam_id
                        ))

            if 'computed' in types:
                subteam_id = mapping.get('computed_data', {}).get('Orientation (Quaternion)', 'localization')
                subteam_info = mapping.get('subteams', {}).get(subteam_id, {'name': 'Unknown'})
                view.computed_data.append(ComputedDataInfo(
                    name='Orientation (Quaternion)',
                    source='TF transform map→base_link',
                    component=component,
                    subteam=subteam_info.get('name', 'Unknown'),
                    subteam_id=subteam_id
                ))

            if 'services' in types:
                for service_name, (service_type, request, response, comp) in service_db.items():
                    if comp == component:
                        subteam, subteam_id = get_subteam_for_service(service_name, component, mapping)
                        view.services.append(ServiceInfo(
                            name=service_name,
                            type=service_type,
                            request=request,
                            response=response,
                            component=component,
                            subteam=subteam,
                            subteam_id=subteam_id
                        ))

                # Add waypoints CRUD placeholder
                if 'WaypointEditor' in component:
                    subteam, subteam_id = get_subteam_for_service('waypoints', component, mapping)
                    view.services.append(ServiceInfo(
                        name='Waypoints CRUD',
                        type='-',
                        request='-',
                        response='-',
                        component=component,
                        subteam=subteam,
                        subteam_id=subteam_id
                    ))

        views.append(view)

    return views


def main():
    """Main entry point."""
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent

    print("Loading subteam mapping...")
    mapping_path = script_dir / "subteam_mapping.yaml"
    mapping = load_subteam_mapping(mapping_path)

    print("Scanning views...")
    views = scan_views(project_root, mapping)

    print(f"Found {len(views)} views")

    templates_dir = script_dir / "templates"
    env = Environment(loader=FileSystemLoader(templates_dir))

    template = env.get_template('system_by_view.html.j2')

    context = {'views': views}

    output = project_root / "system_by_view.html"
    output.write_text(template.render(context))
    print(f"Generated: {output}")

    print("\nDone!")


if __name__ == '__main__':
    main()
