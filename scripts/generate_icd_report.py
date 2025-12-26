#!/usr/bin/env python3
"""
ICD Report Generator for ROS 2 Interfaces

Scans the codebase for ROS publishers, subscribers, and services,
then generates an ICD-compliant report with full message definitions.
"""

import os
import re
import subprocess
import sys
import csv
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Set


class ROSInterface:
    def __init__(
        self,
        subsystem: str,
        producer: str,
        consumer: str,
        iface_type: str,
        name: str,
        msg_type: str,
        frequency: str = "",
        notes: str = "",
    ):
        self.subsystem = subsystem
        self.producer = producer
        self.consumer = consumer
        self.iface_type = iface_type
        self.name = name
        self.msg_type = msg_type
        self.frequency = frequency
        self.notes = notes
        self.definition = None


def get_ros_interface_definition(msg_type: str) -> str:
    """Get the full definition of a ROS message/service using ros2 interface show."""
    try:
        result = subprocess.run(["ros2", "interface", "show", msg_type], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            return result.stdout.strip()
        return f"Error: Could not find definition for {msg_type}"
    except Exception as e:
        return f"Error: {str(e)}"


def scan_python_file(file_path: Path) -> List[Tuple[str, str, str]]:
    """
    Scan a Python file for ROS publishers, subscribers, services, and actions.
    Returns list of (interface_type, topic/service/action_name, msg_type).
    """
    interfaces = []

    try:
        with open(file_path, "r") as f:
            content = f.read()

        # Pattern for create_publisher(MsgType, "/topic_name", qos)
        pub_pattern = r'create_publisher\s*\(\s*(\w+)\s*,\s*["\']([/\w]+)["\']\s*,'
        for match in re.finditer(pub_pattern, content):
            msg_type = match.group(1)
            topic_name = match.group(2)
            interfaces.append(("Publisher", topic_name, msg_type))

        # Pattern for create_subscription(MsgType, "/topic_name", callback, qos)
        sub_pattern = r'create_subscription\s*\(\s*(\w+)\s*,\s*["\']([/\w]+)["\']\s*,'
        for match in re.finditer(sub_pattern, content):
            msg_type = match.group(1)
            topic_name = match.group(2)
            interfaces.append(("Subscriber", topic_name, msg_type))

        # Pattern for create_client(SrvType, "/service_name")
        client_pattern = r'create_client\s*\(\s*(\w+)\s*,\s*["\']([/\w]+)["\']\s*\)'
        for match in re.finditer(client_pattern, content):
            srv_type = match.group(1)
            service_name = match.group(2)
            interfaces.append(("Service Client", service_name, srv_type))

        # Pattern for create_service(SrvType, "/service_name", callback)
        service_pattern = r'create_service\s*\(\s*(\w+)\s*,\s*["\']([/\w]+)["\']\s*,'
        for match in re.finditer(service_pattern, content):
            srv_type = match.group(1)
            service_name = match.group(2)
            interfaces.append(("Service Server", service_name, srv_type))

        # Pattern for ActionServer(node, ActionType, "/action_name", ...)
        action_server_pattern = r'ActionServer\s*\(\s*[^,]+,\s*(\w+)\s*,\s*["\']([/\w]+)["\']\s*,'
        for match in re.finditer(action_server_pattern, content):
            action_type = match.group(1)
            action_name = match.group(2)
            interfaces.append(("Action Server", action_name, action_type))

        # Pattern for ActionClient(node, ActionType, "/action_name")
        action_client_pattern = r'ActionClient\s*\(\s*[^,]+,\s*(\w+)\s*,\s*["\']([/\w]+)["\']\s*\)'
        for match in re.finditer(action_client_pattern, content):
            action_type = match.group(1)
            action_name = match.group(2)
            interfaces.append(("Action Client", action_name, action_type))

    except Exception as e:
        print(f"Error scanning {file_path}: {e}", file=sys.stderr)

    return interfaces


def infer_full_msg_type(short_type: str, imports: Set[str]) -> str:
    """
    Infer the full message type from imports.
    e.g., 'Twist' with 'geometry_msgs.msg' -> 'geometry_msgs/msg/Twist'
    """
    # Check if it's already a full type
    if "/" in short_type:
        return short_type

    # Check common patterns in imports
    for imp in imports:
        if "mrover.msg" in imp and short_type:
            return f"mrover/msg/{short_type}"
        if "mrover.srv" in imp and short_type:
            return f"mrover/srv/{short_type}"
        if "mrover.action" in imp and short_type:
            return f"mrover/action/{short_type}"
        if "geometry_msgs.msg" in imp and short_type in ["Twist", "Vector3", "Point", "Pose"]:
            return f"geometry_msgs/msg/{short_type}"
        if "sensor_msgs.msg" in imp and short_type in ["NavSatFix", "Image", "JointState"]:
            return f"sensor_msgs/msg/{short_type}"
        if "std_msgs.msg" in imp and short_type in ["Header", "String", "Bool"]:
            return f"std_msgs/msg/{short_type}"

    # Default to mrover package
    return f"mrover/msg/{short_type}"


def extract_imports(file_path: Path) -> Set[str]:
    """Extract import statements from a Python file."""
    imports = set()
    try:
        with open(file_path, "r") as f:
            for line in f:
                if line.strip().startswith("from ") or line.strip().startswith("import "):
                    imports.add(line.strip())
    except Exception:
        pass
    return imports


def scan_directory(directory: Path) -> Dict[str, List[Tuple[str, str, str]]]:
    """Scan all Python files in a directory and return discovered interfaces."""
    discovered = {}

    for py_file in directory.rglob("*.py"):
        if "node_modules" in str(py_file) or "__pycache__" in str(py_file):
            continue

        imports = extract_imports(py_file)
        interfaces = scan_python_file(py_file)

        if interfaces:
            # Convert short types to full types
            full_interfaces = []
            for iface_type, name, msg_type in interfaces:
                full_type = infer_full_msg_type(msg_type, imports)
                full_interfaces.append((iface_type, name, full_type))

            discovered[str(py_file)] = full_interfaces

    return discovered


def generate_icd_report(discovered: Dict[str, List[Tuple[str, str, str]]], output_file: Optional[str] = None):
    """Generate an ICD-formatted report from discovered interfaces."""

    # Consolidate interfaces
    topics: Dict[str, Any] = {}
    services: Dict[str, Any] = {}
    actions: Dict[str, Any] = {}

    for file_path, interfaces in discovered.items():
        for iface_type, name, msg_type in interfaces:
            if iface_type in ["Publisher", "Subscriber"]:
                if name not in topics:
                    topics[name] = {"type": msg_type, "publishers": [], "subscribers": []}

                if iface_type == "Publisher":
                    topics[name]["publishers"].append(file_path)
                else:
                    topics[name]["subscribers"].append(file_path)

            elif iface_type in ["Service Client", "Service Server"]:
                if name not in services:
                    services[name] = {"type": msg_type, "clients": [], "servers": []}

                if iface_type == "Service Client":
                    services[name]["clients"].append(file_path)
                else:
                    services[name]["servers"].append(file_path)

            elif iface_type in ["Action Client", "Action Server"]:
                if name not in actions:
                    actions[name] = {"type": msg_type, "clients": [], "servers": []}

                if iface_type == "Action Client":
                    actions[name]["clients"].append(file_path)
                else:
                    actions[name]["servers"].append(file_path)

    # Generate report
    rows = []

    # Header
    rows.append(
        ["Subsystem", "Producer", "Consumer", "Type", "Name", "Frequency", "Definition Link", "Definition", "Notes"]
    )

    # Topics
    print("\n=== ROS TOPICS ===")
    for topic_name, info in sorted(topics.items()):
        msg_type = info["type"]
        definition = get_ros_interface_definition(msg_type)

        producer = "Teleop" if info["publishers"] else "Unknown"
        consumer = "Teleop" if info["subscribers"] else "Unknown"

        print(f"\nTopic: {topic_name}")
        print(f"  Type: {msg_type}")
        print(f"  Publishers: {len(info['publishers'])}")
        print(f"  Subscribers: {len(info['subscribers'])}")

        rows.append(
            [
                "",  # Subsystem - needs manual classification
                producer,
                consumer,
                "Topic",
                topic_name,
                "",  # Frequency - needs manual input
                msg_type,
                definition.replace("\n", "\n"),
                "",  # Notes
            ]
        )

    # Services
    print("\n\n=== ROS SERVICES ===")
    for service_name, info in sorted(services.items()):
        srv_type = info["type"]
        definition = get_ros_interface_definition(srv_type)

        producer = "Teleop" if info["clients"] else "Unknown"
        consumer = "Teleop" if info["servers"] else "Unknown"

        print(f"\nService: {service_name}")
        print(f"  Type: {srv_type}")
        print(f"  Clients: {len(info['clients'])}")
        print(f"  Servers: {len(info['servers'])}")

        rows.append(
            [
                "",  # Subsystem
                producer,
                consumer,
                "Service",
                service_name,
                "",  # Frequency
                srv_type,
                definition.replace("\n", "\n"),
                "",  # Notes
            ]
        )

    # Actions
    print("\n\n=== ROS ACTIONS ===")
    for action_name, info in sorted(actions.items()):
        action_type = info["type"]
        definition = get_ros_interface_definition(action_type)

        producer = "Teleop" if info["clients"] else "Unknown"
        consumer = "Teleop" if info["servers"] else "Unknown"

        print(f"\nAction: {action_name}")
        print(f"  Type: {action_type}")
        print(f"  Clients: {len(info['clients'])}")
        print(f"  Servers: {len(info['servers'])}")

        rows.append(
            [
                "",  # Subsystem
                producer,
                consumer,
                "Action",
                action_name,
                "",  # Frequency
                action_type,
                definition.replace("\n", "\n"),
                "",  # Notes
            ]
        )

    # Write CSV if output file specified
    if output_file:
        with open(output_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerows(rows)
        print(f"\n\nICD report written to: {output_file}")


def main():
    # Get the project root directory
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    teleop_dir = project_root / "teleoperation"

    if not teleop_dir.exists():
        print(f"Error: Teleoperation directory not found at {teleop_dir}")
        sys.exit(1)

    print(f"Scanning directory: {teleop_dir}")
    print("=" * 60)

    # Scan the teleoperation directory
    discovered = scan_directory(teleop_dir)

    print(f"\nScanned {len(discovered)} files")
    print(f"Found interfaces in {sum(1 for v in discovered.values() if v)} files")

    # Generate report
    output_file = None
    if len(sys.argv) > 1:
        output_file = sys.argv[1]

    generate_icd_report(discovered, output_file)

    print("\n" + "=" * 60)
    print("NOTE: Subsystem classification and frequencies need manual review")
    print("Compare this output with your existing icd.csv file")


if __name__ == "__main__":
    main()
