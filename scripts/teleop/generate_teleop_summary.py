#!/usr/bin/env python3
"""
Generate teleoperation system summary documentation.

This script scans the teleoperation backend code and generates:
- system_summary.md: Markdown documentation
- system_summary_tables.html: HTML tables for copying to Google Slides

Output files are placed in the mrover root directory.
"""

import os
import re
from pathlib import Path
from typing import Dict, List, Tuple, Set
from dataclasses import dataclass, field


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
    for line in content.split("\n"):
        # Remove inline comments
        line = line.split("#")[0].strip()
        # Skip empty lines and comment-only lines
        if line:
            lines.append(line)
    return ", ".join(lines)


def parse_srv_file(srv_path: Path) -> Tuple[str, str]:
    """Parse a .srv file and return (request, response) declarations."""
    if not srv_path.exists():
        return "", ""

    content = srv_path.read_text()
    parts = content.split("---")

    request = ""
    response = ""

    if len(parts) >= 1:
        req_lines = []
        for line in parts[0].split("\n"):
            # Remove inline comments
            line = line.split("#")[0].strip()
            if line:
                req_lines.append(line)
        request = ", ".join(req_lines) if req_lines else "-"

    if len(parts) >= 2:
        resp_lines = []
        for line in parts[1].split("\n"):
            # Remove inline comments
            line = line.split("#")[0].strip()
            if line:
                resp_lines.append(line)
        response = ", ".join(resp_lines) if resp_lines else "-"

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


def extract_topics_from_consumer(consumer_path: Path, msg_dir: Path) -> Tuple[List[Topic], List[Topic]]:
    """Extract published and subscribed topics from a consumer file."""
    published = []
    subscribed = []

    content = consumer_path.read_text()

    # Find published topics - look for create_publisher
    pub_pattern = r'self\.\w+\s*=\s*self\.node\.create_publisher\((\w+),\s*["\']([^"\']+)["\']'
    for match in re.finditer(pub_pattern, content):
        msg_type = match.group(1)
        topic_name = match.group(2)

        # Determine full message type
        if "/" in msg_type or msg_type in [
            "Twist",
            "PoseStamped",
            "JointState",
            "Temperature",
            "RelativeHumidity",
            "NavSatFix",
            "Image",
            "Float32",
        ]:
            # Standard ROS or already has package
            full_type = (
                msg_type
                if "/" in msg_type
                else (
                    f"geometry_msgs/{msg_type}"
                    if msg_type in ["Twist", "PoseStamped"]
                    else (
                        f"sensor_msgs/{msg_type}"
                        if msg_type in ["JointState", "Temperature", "RelativeHumidity", "NavSatFix", "Image"]
                        else f"std_msgs/{msg_type}"
                    )
                )
            )
            declaration = get_standard_msg_declaration(full_type)
        else:
            full_type = f"mrover/{msg_type}"
            msg_file = msg_dir / f"{msg_type}.msg"
            declaration = parse_msg_file(msg_file)

        published.append(Topic(topic_name, full_type, declaration))

    # Find subscribed topics - look for forward_ros_topic calls
    sub_pattern = r'await\s+self\.forward_ros_topic\(["\']([^"\']+)["\']\s*,\s*(\w+)\s*,'
    for match in re.finditer(sub_pattern, content):
        topic_name = match.group(1)
        msg_type = match.group(2)

        # Determine full message type
        if "/" in msg_type or msg_type in [
            "Twist",
            "PoseStamped",
            "JointState",
            "Temperature",
            "RelativeHumidity",
            "NavSatFix",
            "Image",
            "Float32",
        ]:
            full_type = (
                msg_type
                if "/" in msg_type
                else (
                    f"geometry_msgs/{msg_type}"
                    if msg_type in ["Twist", "PoseStamped"]
                    else (
                        f"sensor_msgs/{msg_type}"
                        if msg_type in ["JointState", "Temperature", "RelativeHumidity", "NavSatFix", "Image"]
                        else f"std_msgs/{msg_type}"
                    )
                )
            )
            declaration = get_standard_msg_declaration(full_type)
        else:
            full_type = f"mrover/{msg_type}"
            msg_file = msg_dir / f"{msg_type}.msg"
            declaration = parse_msg_file(msg_file)

        subscribed.append(Topic(topic_name, full_type, declaration))

    return published, subscribed


def extract_services_from_views(view_path: Path, srv_dir: Path) -> List[Service]:
    """Extract service calls from REST API view files."""
    services = []

    content = view_path.read_text()

    # Find service definitions - look for create_client
    srv_pattern = r'\.create_client\((\w+),\s*["\']([^"\']+)["\']'
    for match in re.finditer(srv_pattern, content):
        srv_type = match.group(1)
        service_name = match.group(2)

        # Determine full service type and get declarations
        if srv_type == "SetBool":
            full_type = "std_srvs/SetBool"
            request, response = get_standard_srv_declaration(full_type)
        else:
            full_type = f"mrover/{srv_type}"
            srv_file = srv_dir / f"{srv_type}.srv"
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

    # Scan consumers
    consumer_files = {
        "arm": "arm_consumer.py",
        "drive": "drive_consumer.py",
        "science": "science_consumer.py",
        "mast": "mast_consumer.py",
        "nav": "nav_consumer.py",
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
            subscribed_topics=sub,
        )

    # Scan REST API views
    view_files = {
        "science": "science.py",
        "mast": "mast.py",
        "auton": "auton.py",
        "waypoints": "waypoints.py",
    }

    for system_name, filename in view_files.items():
        view_path = views_dir / filename
        if not view_path.exists():
            continue

        services = extract_services_from_views(view_path, srv_dir)

        if system_name in systems:
            # Hybrid system
            systems[system_name].status = "HYBRID"
            systems[system_name].rest_api = filename
            systems[system_name].services = services
        else:
            # REST API only
            systems[system_name] = SystemInfo(
                name=system_name.capitalize(), status="REST API", rest_api=filename, services=services
            )

    return systems


def generate_markdown(systems: Dict[str, SystemInfo], output_path: Path):
    """Generate the markdown documentation."""

    md = ["# Teleoperation ROS Stack System Summary\n"]
    md.append("## System Status Overview\n")
    md.append("| System | Status | Consumer | REST API |")
    md.append("|--------|--------|----------|----------|")

    # Sort systems for consistent output
    for name in ["arm", "drive", "science", "mast", "auton", "nav", "waypoints"]:
        if name in systems:
            sys = systems[name]
            md.append(
                f"| {sys.name} | **{sys.status}** | {sys.consumer if sys.consumer else '-'} | {sys.rest_api if sys.rest_api else '-'} |"
            )

    md.append("\n**Note:** For HYBRID systems - WS: datastream only, everything else on REST API\n")
    md.append("---\n")

    # Generate detailed sections for each system
    for name in ["arm", "drive", "science", "mast", "auton", "nav", "waypoints"]:
        if name not in systems:
            continue

        sys = systems[name]
        md.append(f"## {sys.name} System")
        md.append(f"**Status: {sys.status}**\n")

        if sys.published_topics:
            md.append("### Published Topics")
            md.append("| Topic | Type | Declaration |")
            md.append("|-------|------|-------------|")
            for topic in sys.published_topics:
                md.append(f"| `{topic.name}` | `{topic.msg_type}` | {topic.declaration} |")
            md.append("")

        if sys.subscribed_topics:
            md.append("### Subscribed Topics")
            md.append("| Topic | Type | Declaration |")
            md.append("|-------|------|-------------|")
            for topic in sys.subscribed_topics:
                md.append(f"| `{topic.name}` | `{topic.msg_type}` | {topic.declaration} |")
            md.append("")

        if sys.services:
            md.append("### Services (REST API)")
            md.append("| Service | Type | Request | Response |")
            md.append("|---------|------|---------|----------|")
            for service in sys.services:
                md.append(f"| `{service.name}` | `{service.srv_type}` | {service.request} | {service.response} |")
            md.append("")

        md.append("---\n")

    md.append("**Last Updated:** Auto-generated")

    output_path.write_text("\n".join(md))
    print(f"Generated: {output_path}")


def generate_html(systems: Dict[str, SystemInfo], output_path: Path):
    """Generate the HTML tables for Google Slides."""

    html = ["<!DOCTYPE html>", "<html>", "<head>"]
    html.append('<meta charset="UTF-8">')
    html.append("<title>Teleoperation System Tables</title>")
    html.append("<style>")
    html.append("body { font-family: Arial, sans-serif; padding: 20px; }")
    html.append(
        "table { border-collapse: collapse; width: auto; margin-bottom: 20px; font-size: 9px; table-layout: auto; }"
    )
    html.append(
        "th { background-color: #1a73e8; color: white; padding: 2px 6px; text-align: left; border: 1px solid #ddd; white-space: nowrap; line-height: 1.1; }"
    )
    html.append("td { padding: 2px 6px; border: 1px solid #ddd; vertical-align: top; height: auto; line-height: 1.1; }")
    html.append("tr { height: auto; }")
    html.append("tr:nth-child(even) { background-color: #f2f2f2; }")
    html.append(
        ".table-title { background-color: #1a73e8; color: white; font-weight: bold; text-align: center; padding: 3px 6px; font-size: 10px; line-height: 1.1; }"
    )
    html.append("</style>")
    html.append("</head>")
    html.append("<body>")
    html.append("<h1>Teleoperation ROS Stack System Summary</h1>")

    # System Status Overview
    html.append("<table>")
    html.append('<tr><th colspan="4" class="table-title">System Status Overview</th></tr>')
    html.append("<tr><th>System</th><th>Status</th><th>Consumer</th><th>REST API</th></tr>")

    for name in ["arm", "drive", "science", "mast", "auton", "nav", "waypoints"]:
        if name in systems:
            sys = systems[name]
            html.append(
                f'<tr><td>{sys.name}</td><td>{sys.status}</td><td>{sys.consumer if sys.consumer else "-"}</td><td>{sys.rest_api if sys.rest_api else "-"}</td></tr>'
            )

    html.append("</table>")

    # Detailed tables for each system
    for name in ["arm", "drive", "science", "mast", "auton", "nav"]:
        if name not in systems:
            continue

        sys = systems[name]

        if sys.published_topics:
            html.append("<table>")
            html.append(f'<tr><th colspan="3" class="table-title">{sys.name} - Published Topics</th></tr>')
            html.append("<tr><th>Topic</th><th>Type</th><th>Declaration</th></tr>")
            for topic in sys.published_topics:
                html.append(f"<tr><td>{topic.name}</td><td>{topic.msg_type}</td><td>{topic.declaration}</td></tr>")
            html.append("</table>")

        if sys.subscribed_topics:
            html.append("<table>")
            html.append(f'<tr><th colspan="3" class="table-title">{sys.name} - Subscribed Topics</th></tr>')
            html.append("<tr><th>Topic</th><th>Type</th><th>Declaration</th></tr>")
            for topic in sys.subscribed_topics:
                html.append(f"<tr><td>{topic.name}</td><td>{topic.msg_type}</td><td>{topic.declaration}</td></tr>")
            html.append("</table>")

        if sys.services:
            html.append("<table>")
            html.append(f'<tr><th colspan="4" class="table-title">{sys.name} - Services (REST API)</th></tr>')
            html.append("<tr><th>Service</th><th>Type</th><th>Request</th><th>Response</th></tr>")
            for service in sys.services:
                html.append(
                    f"<tr><td>{service.name}</td><td>{service.srv_type}</td><td>{service.request}</td><td>{service.response}</td></tr>"
                )
            html.append("</table>")

    html.append("</body>")
    html.append("</html>")

    output_path.write_text("\n".join(html))
    print(f"Generated: {output_path}")


def main():
    """Main entry point."""
    # Find project root (assumes script is in scripts/)
    script_dir = Path(__file__).parent
    project_root = script_dir.parent

    print("Scanning teleoperation system...")
    systems = scan_teleoperation_system(project_root)

    print(f"Found {len(systems)} systems")

    # Generate outputs
    md_output = project_root / "system_summary.md"
    html_output = project_root / "system_summary_tables.html"

    generate_markdown(systems, md_output)
    generate_html(systems, html_output)

    print("\nDone!")


if __name__ == "__main__":
    main()
