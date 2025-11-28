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
import json
from pathlib import Path
from typing import Dict, List, Tuple, Set, Optional
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
    with open(mapping_path, "r") as f:
        return yaml.safe_load(f)


def parse_msg_file(msg_path: Path) -> str:
    """Parse a .msg file and return its declaration."""
    if not msg_path.exists():
        return ""

    content = msg_path.read_text()
    lines = []
    for line in content.split("\n"):
        line = line.split("#")[0].strip()
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
            line = line.split("#")[0].strip()
            if line:
                req_lines.append(line)
        request = ", ".join(req_lines) if req_lines else "-"

    if len(parts) >= 2:
        resp_lines = []
        for line in parts[1].split("\n"):
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


def get_msg_declaration(msg_type: str, msg_dir: Path) -> str:
    """Get message declaration for any message type."""
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
        return get_standard_msg_declaration(full_type)
    else:
        msg_file = msg_dir / f"{msg_type}.msg"
        return parse_msg_file(msg_file)


def get_subteam_for_topic(topic_name: str, component: str, mapping: Dict) -> Tuple[str, str]:
    """Get subteam name and ID for a topic."""
    subteam_id = mapping.get("topics", {}).get(topic_name)
    if not subteam_id:
        subteam_id = mapping.get("components", {}).get(component, "esw")

    subteam_info = mapping.get("subteams", {}).get(subteam_id, {"name": "Unknown"})
    return subteam_info.get("name", "Unknown"), subteam_id


def get_subteam_for_service(service_name: str, component: str, mapping: Dict) -> Tuple[str, str]:
    """Get subteam name and ID for a service."""
    subteam_id = mapping.get("services", {}).get(service_name)
    if not subteam_id:
        subteam_id = mapping.get("components", {}).get(component, "esw")

    subteam_info = mapping.get("subteams", {}).get(subteam_id, {"name": "Unknown"})
    return subteam_info.get("name", "Unknown"), subteam_id


def scan_consumer_file(consumer_path: Path) -> Dict[str, List[Tuple[str, str, str]]]:
    """
    Scan a consumer Python file to extract ROS topics and their GUI message types.
    Returns dict mapping websocket name to list of (topic_name, topic_type, gui_msg_type, direction).
    """
    if not consumer_path.exists():
        return {}

    content = consumer_path.read_text()
    result = {}

    # Extract websocket name from filename (e.g., arm_consumer.py -> arm)
    websocket_name = consumer_path.stem.replace("_consumer", "")
    result[websocket_name] = []

    # Find forward_ros_topic calls (subscriptions)
    forward_pattern = r'await\s+self\.forward_ros_topic\(["\']([^"\']+)["\']\s*,\s*(\w+)\s*,\s*["\']([^"\']+)["\']\)'
    for match in re.finditer(forward_pattern, content):
        topic_name, topic_type, gui_msg_type = match.groups()
        result[websocket_name].append((topic_name, topic_type, gui_msg_type, "subscribed"))

    # Find send_json calls with "type" field (for computed data like orientation, calibration)
    send_json_pattern = r'self\.send_json\(\s*\{[^}]*["\']type["\']\s*:\s*["\']([^"\']+)["\']'
    for match in re.finditer(send_json_pattern, content):
        gui_msg_type = match.group(1)
        # Mark as subscribed with special topic name to indicate it's computed
        result[websocket_name].append((f"[computed:{gui_msg_type}]", "Computed", gui_msg_type, "subscribed"))

    # Build mapping of publisher variable names to (topic_name, topic_type)
    publishers = {}
    pub_pattern = r'self\.(\w+)\s*=\s*self\.node\.create_publisher\((\w+)\s*,\s*["\']([^"\']+)["\']\s*,'
    for match in re.finditer(pub_pattern, content):
        pub_var, topic_type, topic_name = match.groups()
        publishers[pub_var] = (topic_name, topic_type)

    # Find receive_json match cases to map GUI message type to publisher
    # Pattern: case { "type": "gui_msg_type", ...}: ... self.pub_var.publish(...) or function(self.pub_var)
    receive_section = re.search(r"async def receive_json.*?(?=\n    async def|\Z)", content, re.DOTALL)
    if receive_section:
        receive_content = receive_section.group(0)
        # Find case statements with type
        case_pattern = r'case\s*\{[^}]*["\']type["\']\s*:\s*["\']([^"\']+)["\'][^}]*\}:(.*?)(?=case\s*\{|case\s*_:|\Z)'
        for match in re.finditer(case_pattern, receive_content, re.DOTALL):
            gui_msg_type, case_body = match.groups()
            # Find direct publisher calls: self.pub_var.publish(...)
            pub_call_pattern = r"self\.(\w+)\.publish\("
            for pub_match in re.finditer(pub_call_pattern, case_body):
                pub_var = pub_match.group(1)
                if pub_var in publishers:
                    topic_name, topic_type = publishers[pub_var]
                    result[websocket_name].append((topic_name, topic_type, gui_msg_type, "published"))

            # Find publisher passed to helper function: function(..., self.pub_var, ...)
            for pub_var in publishers.keys():
                if f"self.{pub_var}" in case_body:
                    topic_name, topic_type = publishers[pub_var]
                    result[websocket_name].append((topic_name, topic_type, gui_msg_type, "published"))

    return result


def scan_vue_component(component_path: Path) -> Dict[str, any]:
    """
    Scan a Vue component file to extract websocket message types it listens to.
    Returns dict with component info.
    """
    if not component_path.exists():
        return {}

    content = component_path.read_text()
    result = {
        "name": component_path.stem,
        "subscribes_to": [],
        "publishes_to": [],
    }

    # Find watch statements for websocket messages
    watch_pattern = r'watch\(\s*\w+Message\s*,.*?msg\.type\s*===?\s*["\']([^"\']+)["\']'
    for match in re.finditer(watch_pattern, content, re.DOTALL):
        msg_type = match.group(1)
        result["subscribes_to"].append(msg_type)

    # Find computed message type checks
    computed_pattern = r'\.type\s*===?\s*["\']([^"\']+)["\']'
    for match in re.finditer(computed_pattern, content):
        msg_type = match.group(1)
        if msg_type not in result["subscribes_to"]:
            result["subscribes_to"].append(msg_type)

    # Find sendMessage calls (publications)
    send_pattern = r'websocketStore\.sendMessage\(["\'](\w+)["\']\s*,\s*\{[^}]*type:\s*["\']([^"\']+)["\']'
    for match in re.finditer(send_pattern, content):
        websocket, msg_type = match.groups()
        result["publishes_to"].append((websocket, msg_type))

    return result


def scan_vue_view(view_path: Path) -> Dict[str, any]:
    """
    Scan a Vue view file to extract components used and websockets connected.
    """
    if not view_path.exists():
        return {}

    content = view_path.read_text()
    result = {
        "name": view_path.stem,
        "websockets": [],
        "components": {},
    }

    # Find setupWebSocket calls with string literals
    websocket_pattern = r'websocketStore\.setupWebSocket\(["\'](\w+)["\']\)'
    for match in re.finditer(websocket_pattern, content):
        websocket = match.group(1)
        result["websockets"].append(websocket)

    # Find websocket arrays like: const topics = ['drive', 'nav', 'science', 'mast']
    # followed by setupWebSocket(topic) in a loop
    array_pattern = r"const\s+\w+\s*=\s*\[([\s\S]*?)\]"
    for match in re.finditer(array_pattern, content):
        array_content = match.group(1)
        # Extract string literals from the array
        string_pattern = r'["\'](\w+)["\']'
        for str_match in re.finditer(string_pattern, array_content):
            websocket = str_match.group(1)
            # Check if this array is used with setupWebSocket
            if "setupWebSocket" in content and websocket not in result["websockets"]:
                result["websockets"].append(websocket)

    # Find component usage with msg-type prop
    # Match each component tag with msg-type attribute
    component_pattern = r'<(\w+)[^>]*?msg-type=["\']([^"\']+)["\'][^>]*?>'
    for match in re.finditer(component_pattern, content):
        component_name, msg_type = match.groups()
        if component_name not in result["components"]:
            result["components"][component_name] = {"msg_types": []}
        if msg_type not in result["components"][component_name]["msg_types"]:
            result["components"][component_name]["msg_types"].append(msg_type)

    # Find all components used (even without props)
    component_tag_pattern = r"<([A-Z]\w+)"
    for match in re.finditer(component_tag_pattern, content):
        component_name = match.group(1)
        if component_name not in result["components"]:
            result["components"][component_name] = {"msg_types": []}

    return result


def build_topic_database(project_root: Path) -> Dict[str, Tuple[str, str, str]]:
    """
    Build a database of all ROS topics by scanning consumer files.
    Returns dict mapping gui_msg_type to (topic_name, topic_type, direction).
    """
    topic_db = {}
    consumers_dir = project_root / "teleoperation/basestation_gui/backend/consumers"

    for consumer_file in consumers_dir.glob("*_consumer.py"):
        consumer_data = scan_consumer_file(consumer_file)
        for websocket_name, topics in consumer_data.items():
            for topic_name, topic_type, gui_msg_type, direction in topics:
                # Map GUI message type to topic info
                topic_db[gui_msg_type] = (topic_name, topic_type, direction)

    return topic_db


def scan_views(project_root: Path, mapping: Dict) -> List[ViewInfo]:
    """Scan frontend views and extract information for each view."""
    views = []

    # Scan all consumers to build topic database
    topic_db = build_topic_database(project_root)

    # Scan backend consumers
    consumers_dir = project_root / "teleoperation/basestation_gui/backend/consumers"
    consumer_topics = {}
    for consumer_file in consumers_dir.glob("*_consumer.py"):
        data = scan_consumer_file(consumer_file)
        consumer_topics.update(data)

    # Scan frontend components
    components_dir = project_root / "teleoperation/basestation_gui/frontend/src/components"
    component_info = {}
    for component_file in components_dir.glob("*.vue"):
        info = scan_vue_component(component_file)
        if info:
            component_info[info["name"]] = info

    # Scan frontend views
    views_dir = project_root / "teleoperation/basestation_gui/frontend/src/views"

    view_files = [
        ("AutonTask.vue", "Autonomy Mission", "/AutonTask"),
        ("DMTask.vue", "Delivery Mission", "/DMTask"),
        ("ESTask.vue", "Equipment Servicing", "/ESTask"),
        ("SPTask.vue", "Science Payload", "/SPTask"),
    ]

    service_db = {
        "/enable_auton": (
            "mrover/EnableAuton",
            "bool enable, GPSWaypoint[] waypoints",
            "bool success",
            "AutonWaypointEditor",
        ),
        "/enable_teleop": ("std_srvs/SetBool", "bool data", "bool success, string message", "AutonWaypointEditor"),
        "/sp_funnel_set_position": (
            "mrover/ServoSetPos",
            "float32 position, bool is_counterclockwise",
            "bool success",
            "HexHub",
        ),
        "/gimbal_set_position": (
            "mrover/ServoSetPos",
            "float32 position, bool is_counterclockwise",
            "bool success",
            "GimbalControls",
        ),
        "/panorama/start": ("mrover/PanoramaStart", "-", "-", "PanoCam"),
        "/panorama/end": ("mrover/PanoramaEnd", "-", "bool success, sensor_msgs/Image img", "PanoCam"),
    }

    for view_filename, description, route in view_files:
        view_path = views_dir / view_filename
        if not view_path.exists():
            continue

        view_data = scan_vue_view(view_path)

        view = ViewInfo(
            name=view_data["name"], description=description, route=route, websockets=view_data["websockets"]
        )

        # For each component used in the view, collect topics
        for comp_name, comp_data in view_data["components"].items():
            comp_lower = comp_name

            # Get component's message subscriptions
            if comp_name in component_info:
                comp = component_info[comp_name]

                # Add subscribed topics based on what message types this view instance uses
                msg_types_for_view = comp_data.get("msg_types", [])
                if not msg_types_for_view:
                    msg_types_for_view = comp["subscribes_to"]

                for msg_type in msg_types_for_view:
                    # Look up topic info from consumer data
                    for websocket in view_data["websockets"]:
                        if websocket in consumer_topics:
                            for topic_name, topic_type, gui_msg_type, direction in consumer_topics[websocket]:
                                if gui_msg_type == msg_type and direction == "subscribed":
                                    decl = get_msg_declaration(topic_type, project_root / "msg")
                                    subteam, subteam_id = get_subteam_for_topic(topic_name, comp_name, mapping)
                                    # Check if already added to avoid duplicates
                                    already_exists = any(
                                        t.name == topic_name and t.component == comp_name
                                        for t in view.subscribed_topics
                                    )
                                    if not already_exists:
                                        view.subscribed_topics.append(
                                            TopicInfo(
                                                name=topic_name,
                                                type=topic_type,
                                                declaration=decl,
                                                component=comp_name,
                                                subteam=subteam,
                                                subteam_id=subteam_id,
                                            )
                                        )

                # Add published topics
                for websocket, msg_type in comp["publishes_to"]:
                    if websocket in view_data["websockets"]:
                        if websocket in consumer_topics:
                            for topic_name, topic_type, gui_msg_type, direction in consumer_topics[websocket]:
                                if gui_msg_type == msg_type and direction == "published":
                                    decl = get_msg_declaration(topic_type, project_root / "msg")
                                    subteam, subteam_id = get_subteam_for_topic(topic_name, comp_name, mapping)
                                    # Check if already added to avoid duplicates
                                    already_exists = any(
                                        t.name == topic_name and t.component == comp_name for t in view.published_topics
                                    )
                                    if not already_exists:
                                        view.published_topics.append(
                                            TopicInfo(
                                                name=topic_name,
                                                type=topic_type,
                                                declaration=decl,
                                                component=comp_name,
                                                subteam=subteam,
                                                subteam_id=subteam_id,
                                            )
                                        )

        # Add services used in view
        for service_name, (service_type, request, response, comp) in service_db.items():
            if comp in view_data["components"]:
                subteam, subteam_id = get_subteam_for_service(service_name, comp, mapping)
                view.services.append(
                    ServiceInfo(
                        name=service_name,
                        type=service_type,
                        request=request,
                        response=response,
                        component=comp,
                        subteam=subteam,
                        subteam_id=subteam_id,
                    )
                )

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

    template = env.get_template("system_by_view.html.j2")

    context = {"views": views}

    output = project_root / "system_by_view.html"
    output.write_text(template.render(context))
    print(f"Generated: {output}")

    print("\nDone!")


if __name__ == "__main__":
    main()
