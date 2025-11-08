#!/usr/bin/env python3
"""
Generate teleoperation system summary documentation using Jinja templates.

This script scans the teleoperation backend code and generates:
- system.html: HTML tables for copying to Google Slides

Output files are placed in the mrover root directory.
"""

import os
import re
from pathlib import Path
from typing import Dict, List, Tuple, Set
from dataclasses import dataclass, field
from jinja2 import Environment, FileSystemLoader


@dataclass
class Topic:
    name: str
    msg_type: str
    declaration: str = ""


@dataclass
class Service:
    name: str
    srv_type: str
    request: str = ""
    response: str = ""


@dataclass
class SystemInfo:
    name: str
    status: str  # WEBSOCKET, REST API, HYBRID
    consumer: str = ""
    rest_api: str = ""
    published_topics: List[Topic] = field(default_factory=list)
    subscribed_topics: List[Topic] = field(default_factory=list)
    services: List[Service] = field(default_factory=list)


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


def parse_action_file(action_path: Path) -> Tuple[str, str, str]:
    """Parse a .action file and return (goal, result, feedback) declarations."""
    if not action_path.exists():
        return "", "", ""

    content = action_path.read_text()
    parts = content.split('---')

    goal = ""
    result = ""
    feedback = ""

    if len(parts) >= 1:
        goal_lines = []
        for line in parts[0].split('\n'):
            line = line.split('#')[0].strip()
            if line:
                goal_lines.append(line)
        goal = ', '.join(goal_lines) if goal_lines else "-"

    if len(parts) >= 2:
        result_lines = []
        for line in parts[1].split('\n'):
            line = line.split('#')[0].strip()
            if line:
                result_lines.append(line)
        result = ', '.join(result_lines) if result_lines else "-"

    if len(parts) >= 3:
        feedback_lines = []
        for line in parts[2].split('\n'):
            line = line.split('#')[0].strip()
            if line:
                feedback_lines.append(line)
        feedback = ', '.join(feedback_lines) if feedback_lines else "-"

    return goal, result, feedback


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


def extract_topics_from_consumer(consumer_path: Path, msg_dir: Path) -> Tuple[List[Topic], List[Topic]]:
    """Extract published and subscribed topics from a consumer file."""
    published = []
    subscribed = []

    content = consumer_path.read_text()

    pub_pattern = r'self\.\w+\s*=\s*self\.node\.create_publisher\((\w+),\s*["\']([^"\']+)["\']'
    for match in re.finditer(pub_pattern, content):
        msg_type = match.group(1)
        topic_name = match.group(2)

        if '/' in msg_type or msg_type in ['Twist', 'PoseStamped', 'JointState', 'Temperature', 'RelativeHumidity', 'NavSatFix', 'Image', 'Float32']:
            full_type = msg_type if '/' in msg_type else f'geometry_msgs/{msg_type}' if msg_type in ['Twist', 'PoseStamped'] else f'sensor_msgs/{msg_type}' if msg_type in ['JointState', 'Temperature', 'RelativeHumidity', 'NavSatFix', 'Image'] else f'std_msgs/{msg_type}'
            declaration = get_standard_msg_declaration(full_type)
        else:
            full_type = f'mrover/{msg_type}'
            msg_file = msg_dir / f'{msg_type}.msg'
            declaration = parse_msg_file(msg_file)

        published.append(Topic(topic_name, full_type, declaration))

    sub_pattern = r'await\s+self\.forward_ros_topic\(["\']([^"\']+)["\']\s*,\s*(\w+)\s*,'
    for match in re.finditer(sub_pattern, content):
        topic_name = match.group(1)
        msg_type = match.group(2)

        if '/' in msg_type or msg_type in ['Twist', 'PoseStamped', 'JointState', 'Temperature', 'RelativeHumidity', 'NavSatFix', 'Image', 'Float32']:
            full_type = msg_type if '/' in msg_type else f'geometry_msgs/{msg_type}' if msg_type in ['Twist', 'PoseStamped'] else f'sensor_msgs/{msg_type}' if msg_type in ['JointState', 'Temperature', 'RelativeHumidity', 'NavSatFix', 'Image'] else f'std_msgs/{msg_type}'
            declaration = get_standard_msg_declaration(full_type)
        else:
            full_type = f'mrover/{msg_type}'
            msg_file = msg_dir / f'{msg_type}.msg'
            declaration = parse_msg_file(msg_file)

        subscribed.append(Topic(topic_name, full_type, declaration))

    return published, subscribed


def extract_services_from_views(view_path: Path, srv_dir: Path) -> List[Service]:
    """Extract service calls from REST API view files."""
    services = []

    content = view_path.read_text()

    srv_pattern = r'\.create_client\((\w+),\s*["\']([^"\']+)["\']'
    for match in re.finditer(srv_pattern, content):
        srv_type = match.group(1)
        service_name = match.group(2)

        if srv_type == 'SetBool':
            full_type = 'std_srvs/SetBool'
            request, response = get_standard_srv_declaration(full_type)
        else:
            full_type = f'mrover/{srv_type}'
            srv_file = srv_dir / f'{srv_type}.srv'
            request, response = parse_srv_file(srv_file)

        services.append(Service(service_name, full_type, request, response))

    return services


def scan_teleoperation_system(project_root: Path) -> Dict[str, SystemInfo]:
    """Scan the teleoperation backend and extract system information."""
    backend_dir = project_root / "teleoperation/basestation_gui/backend"
    consumers_dir = backend_dir / "consumers"
    views_dir = backend_dir / "api/views"
    msg_dir = project_root / "msg"
    srv_dir = project_root / "srv"

    systems = {}

    consumer_files = {
        'arm': 'arm_consumer.py',
        'drive': 'drive_consumer.py',
        'science': 'science_consumer.py',
        'mast': 'mast_consumer.py',
        'nav': 'nav_consumer.py',
    }

    for system_name, filename in consumer_files.items():
        consumer_path = consumers_dir / filename
        if not consumer_path.exists():
            continue

        pub, sub = extract_topics_from_consumer(consumer_path, msg_dir)

        systems[system_name] = SystemInfo(
            name=system_name.capitalize(),
            status="WEBSOCKET",
            consumer=filename,
            published_topics=pub,
            subscribed_topics=sub
        )

    view_files = {
        'science': 'science.py',
        'mast': 'mast.py',
        'auton': 'auton.py',
        'waypoints': 'waypoints.py',
    }

    for system_name, filename in view_files.items():
        view_path = views_dir / filename
        if not view_path.exists():
            continue

        services = extract_services_from_views(view_path, srv_dir)

        if system_name in systems:
            systems[system_name].status = "HYBRID"
            systems[system_name].rest_api = filename
            systems[system_name].services = services
        else:
            systems[system_name] = SystemInfo(
                name=system_name.capitalize(),
                status="REST API",
                rest_api=filename,
                services=services
            )

    return systems


def main():
    """Main entry point."""
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent

    print("Scanning teleoperation system...")
    systems = scan_teleoperation_system(project_root)

    print(f"Found {len(systems)} systems")

    templates_dir = script_dir / "templates"
    env = Environment(loader=FileSystemLoader(templates_dir))

    html_template = env.get_template('system_summary_tables.html.j2')

    context = {'systems': systems}

    html_output = project_root / "system.html"

    html_output.write_text(html_template.render(context))
    print(f"Generated: {html_output}")

    print("\nDone!")


if __name__ == '__main__':
    main()
