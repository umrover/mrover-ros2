#!/usr/bin/env python3
"""
ICD Compliance Checker

Validates the codebase against the ICD (Interface Control Document) specification.
Checks message definitions, topic names, and service definitions.

Usage:
    icd_check.py <icd.csv>          Check compliance
    icd_check.py <icd.csv> --fix    Check and create missing message types
"""

import argparse
import csv
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class ICDEntry:
    subsystem: str
    producer: str
    consumer: str
    interface_type: str  # Topic, Service, Action
    name: str
    frequency: str
    msg_type: str
    definition: str
    notes: str


class ICDChecker:
    def __init__(self, mrover_root: Path, icd_path: Path, fix: bool = False):
        self.root = mrover_root
        self.icd_path = icd_path
        self.fix = fix
        self.entries: list[ICDEntry] = []
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.missing_types: list[tuple[str, str, str, str]] = []  # (pkg, typ, iface_type, definition)

    def parse_icd(self) -> None:
        with open(self.icd_path, "r") as f:
            reader = csv.reader(f)
            rows = list(reader)

        header_idx = None
        for i, row in enumerate(rows):
            if row and row[0] == "Subsystem":
                header_idx = i
                break

        if header_idx is None:
            self.errors.append("Could not find header row in ICD CSV")
            return

        for row in rows[header_idx + 1 :]:
            if not row or not row[0].strip():
                continue
            if len(row) < 5:
                continue

            name = row[4].strip() if len(row) > 4 else ""
            if not name or name.startswith("/delta"):
                continue

            self.entries.append(
                ICDEntry(
                    subsystem=row[0].strip() if len(row) > 0 else "",
                    producer=row[1].strip() if len(row) > 1 else "",
                    consumer=row[2].strip() if len(row) > 2 else "",
                    interface_type=row[3].strip() if len(row) > 3 else "",
                    name=name,
                    frequency=row[5].strip() if len(row) > 5 else "",
                    msg_type=row[6].strip() if len(row) > 6 else "",
                    definition=row[7].strip() if len(row) > 7 else "",
                    notes=row[8].strip() if len(row) > 8 else "",
                )
            )

    def normalize_msg_type(self, msg_type: str) -> Optional[tuple[str, str]]:
        """Returns (package, type) or None if unparseable."""
        msg_type = msg_type.strip()
        if not msg_type:
            return None

        # Fix common typos
        msg_type = msg_type.replace("gemetry_msgs", "geometry_msgs")

        if "/" in msg_type:
            parts = msg_type.split("/")
            pkg = parts[0]
            typ = parts[-1]
            return (pkg, typ)
        return None

    def check_msg_exists(self, pkg: str, typ: str) -> bool:
        """Check if a message type exists."""
        if pkg == "mrover":
            msg_file = self.root / "msg" / f"{typ}.msg"
            srv_file = self.root / "srv" / f"{typ}.srv"
            action_file = self.root / "action" / f"{typ}.action"
            return msg_file.exists() or srv_file.exists() or action_file.exists()
        return True

    def parse_msg_file(self, path: Path) -> list[str]:
        """Parse a .msg file and return normalized field definitions."""
        if not path.exists():
            return []
        fields = []
        with open(path, "r") as f:
            for line in f:
                line = line.split("#")[0].strip()
                if not line:
                    continue
                line = re.sub(r"\[\d*\]", "[]", line)
                fields.append(line)
        return fields

    def normalize_icd_definition(self, definition: str) -> list[str]:
        """Normalize ICD definition field for comparison."""
        fields = []
        for line in definition.split("\n"):
            line = line.strip()
            if not line:
                continue
            if line.startswith("---"):
                continue
            if line.startswith("#"):
                continue
            if ":" in line and not any(line.startswith(t) for t in ["uint8", "int8", "uint16", "int16", "uint32", "int32", "uint64", "int64", "float32", "float64"]):
                continue
            line = re.sub(r"\[\d*\]", "[]", line)
            fields.append(line)
        return fields

    def format_definition_for_file(self, definition: str, interface_type: str) -> str:
        """Convert ICD definition to proper .msg/.srv/.action format."""
        lines = []
        in_response = False
        in_feedback = False

        for line in definition.split("\n"):
            line = line.strip()
            if not line:
                continue

            if line == "---":
                lines.append("---")
                if not in_response:
                    in_response = True
                else:
                    in_feedback = True
                continue

            # Skip inline type definitions (like "ImageTarget:")
            if line.endswith(":") and not "=" in line:
                continue

            # Fix common type issues
            line = line.replace("double ", "float64 ")
            line = line.replace("char[]", "string")

            # Handle constant definitions
            if "=" in line:
                lines.append(line)
                continue

            # Regular field
            lines.append(line)

        return "\n".join(lines) + "\n"

    def create_missing_type(self, pkg: str, typ: str, interface_type: str, definition: str) -> bool:
        """Create a stub message/service/action file."""
        if pkg != "mrover":
            return False

        if interface_type == "Topic":
            dir_path = self.root / "msg"
            ext = ".msg"
        elif interface_type == "Service":
            dir_path = self.root / "srv"
            ext = ".srv"
        elif interface_type == "Action":
            dir_path = self.root / "action"
            ext = ".action"
        else:
            return False

        file_path = dir_path / f"{typ}{ext}"

        if file_path.exists():
            return False

        dir_path.mkdir(parents=True, exist_ok=True)

        content = self.format_definition_for_file(definition, interface_type)
        if not content.strip():
            content = "# TODO: Define fields\n"

        file_path.write_text(content)
        print(f"  Created: {file_path.relative_to(self.root)}")
        return True

    def check_message_definitions(self) -> None:
        """Check that message types exist and fields match."""
        for entry in self.entries:
            if not entry.msg_type:
                self.warnings.append(
                    f"[{entry.name}] No message type specified in ICD"
                )
                continue

            parsed = self.normalize_msg_type(entry.msg_type)
            if not parsed:
                self.warnings.append(
                    f"[{entry.name}] Could not parse message type: {entry.msg_type}"
                )
                continue

            pkg, typ = parsed

            if not self.check_msg_exists(pkg, typ):
                self.errors.append(
                    f"[{entry.name}] Message type not found: {pkg}/{typ}"
                )
                self.missing_types.append((pkg, typ, entry.interface_type, entry.definition))
                continue

            if pkg == "mrover" and entry.definition:
                if entry.interface_type == "Topic":
                    msg_path = self.root / "msg" / f"{typ}.msg"
                elif entry.interface_type == "Service":
                    msg_path = self.root / "srv" / f"{typ}.srv"
                else:
                    continue

                if msg_path.exists():
                    actual_fields = self.parse_msg_file(msg_path)
                    expected_fields = self.normalize_icd_definition(entry.definition)

                    for expected in expected_fields:
                        if not any(expected in actual for actual in actual_fields):
                            field_name = expected.split()[-1] if expected.split() else ""
                            if field_name and not any(
                                field_name in actual for actual in actual_fields
                            ):
                                self.warnings.append(
                                    f"[{entry.name}] Field mismatch in {typ}: "
                                    f"expected '{expected}'"
                                )

    def check_topic_usage(self) -> None:
        """Grep source code for topic usage."""
        topic_entries = [e for e in self.entries if e.interface_type == "Topic"]

        source_topics: set[str] = set()
        for pattern in ["**/*.cpp", "**/*.py", "**/*.vue", "**/*.ts"]:
            for f in self.root.glob(pattern):
                if "build" in str(f) or "install" in str(f):
                    continue
                try:
                    content = f.read_text()
                    matches = re.findall(r'["\'](/[a-z_/]+)["\']', content, re.IGNORECASE)
                    source_topics.update(matches)
                except Exception:
                    pass

        icd_topics = {e.name for e in topic_entries}

        for topic in icd_topics:
            if topic not in source_topics:
                self.warnings.append(
                    f"[{topic}] Defined in ICD but not found in source code"
                )

    def run(self) -> bool:
        """Run all checks. Returns True if no errors."""
        print(f"Parsing ICD from {self.icd_path}...")
        self.parse_icd()
        print(f"Found {len(self.entries)} ICD entries\n")

        print("Checking message definitions...")
        self.check_message_definitions()

        print("Checking topic usage in source...\n")
        self.check_topic_usage()

        if self.errors:
            print("=" * 60)
            print(f"ERRORS ({len(self.errors)}):")
            print("=" * 60)
            for e in self.errors:
                print(f"  {e}")
            print()

        if self.warnings:
            print("=" * 60)
            print(f"WARNINGS ({len(self.warnings)}):")
            print("=" * 60)
            for w in self.warnings:
                print(f"  {w}")
            print()

        if not self.errors and not self.warnings:
            print("All checks passed!")

        print(f"\nSummary: {len(self.errors)} errors, {len(self.warnings)} warnings")

        if self.fix and self.missing_types:
            print("\n" + "=" * 60)
            print("FIXING: Creating missing message types...")
            print("=" * 60)
            created = 0
            for pkg, typ, iface_type, definition in self.missing_types:
                if self.create_missing_type(pkg, typ, iface_type, definition):
                    created += 1
            print(f"\nCreated {created} files. Remember to add them to CMakeLists.txt!")

        return len(self.errors) == 0


def main():
    parser = argparse.ArgumentParser(
        description="Check codebase compliance against ICD specification"
    )
    parser.add_argument(
        "icd_file",
        type=Path,
        help="Path to the ICD CSV file"
    )
    parser.add_argument(
        "--fix",
        action="store_true",
        help="Create stub files for missing message types"
    )
    args = parser.parse_args()

    mrover_root = Path(__file__).parent.parent.parent

    if not args.icd_file.exists():
        print(f"ICD file not found: {args.icd_file}")
        sys.exit(1)

    checker = ICDChecker(mrover_root, args.icd_file, fix=args.fix)
    success = checker.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
