#!/usr/bin/env python3
"""
ICD Compliance Checker

Validates the codebase against the ICD specification.
Checks message definitions for exact match with ICD.

Download ICD as CSV format, assuming ICD format of year 25-26

Usage:
    icd_check.py <icd.csv>                  Check compliance
    icd_check.py <icd.csv> --fix            Fix all mismatches
    icd_check.py <icd.csv> --fix /topic     Fix specific topic only
"""

import argparse
import csv
import re
import sys
from dataclasses import dataclass, field
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


@dataclass
class Mismatch:
    topic: str
    msg_type: str
    file_path: Path
    interface_type: str
    expected_content: str
    actual_content: str
    missing_fields: list[str] = field(default_factory=list)
    extra_fields: list[str] = field(default_factory=list)


class ICDChecker:
    def __init__(self, mrover_root: Path, icd_path: Path, fix: Optional[str] = None):
        self.root = mrover_root
        self.icd_path = icd_path
        self.fix = fix  # None = no fix, "" = fix all, "/topic" = fix specific
        self.entries: list[ICDEntry] = []
        self.missing: list[str] = []
        self.mismatches: list[Mismatch] = []
        self.missing_types: list[tuple[str, str, str, str, str]] = []  # (pkg, typ, iface_type, definition, topic)

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
            self.missing.append("Could not find header row in ICD CSV")
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
        """Parse a .msg/.srv file and return normalized field definitions."""
        if not path.exists():
            return []
        fields = []
        with open(path, "r") as f:
            for line in f:
                line = line.split("#")[0].strip()
                if not line:
                    continue
                if line == "---":
                    fields.append("---")
                    continue
                line = re.sub(r"\[\d*\]", "[]", line)
                fields.append(line)
        return fields

    def normalize_icd_field(self, line: str) -> Optional[str]:
        """Normalize a single ICD field line. Returns None if not a valid field."""
        line = line.strip()
        if not line:
            return None
        if line.startswith("#"):
            return None
        if line.endswith(":") and "=" not in line:
            return None
        if line == "---":
            return "---"

        line = line.replace("double ", "float64 ")
        line = line.replace("char[]", "string")
        line = re.sub(r"\[\d*\]", "[]", line)

        # Add default field names for types without them
        parts = line.split()
        if len(parts) == 1 and "/" in parts[0]:
            # Type without field name, e.g., "std_msgs/Header" -> "std_msgs/Header header"
            type_name = parts[0].split("/")[-1].lower()
            line = f"{parts[0]} {type_name}"

        return line

    def normalize_icd_definition(self, definition: str) -> list[str]:
        """Normalize ICD definition field for comparison."""
        fields = []
        for line in definition.split("\n"):
            normalized = self.normalize_icd_field(line)
            if normalized:
                fields.append(normalized)
        return fields

    def format_definition_for_file(self, definition: str) -> str:
        """Convert ICD definition to proper .msg/.srv/.action format."""
        lines = []
        for line in definition.split("\n"):
            normalized = self.normalize_icd_field(line)
            if normalized:
                lines.append(normalized)
        return "\n".join(lines) + "\n"

    def should_fix(self, topic: str) -> bool:
        """Check if this topic should be fixed."""
        if self.fix is None:
            return False
        if self.fix == "":
            return True
        return self.fix == topic

    def create_missing_type(self, pkg: str, typ: str, interface_type: str, definition: str) -> bool:
        """Create a message/service/action file."""
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

        content = self.format_definition_for_file(definition)
        if not content.strip():
            content = "# TODO: Define fields\n"

        file_path.write_text(content)
        print(f"  Created: {file_path.relative_to(self.root)}")
        return True

    def fix_mismatch(self, mismatch: Mismatch) -> bool:
        """Overwrite file to match ICD exactly."""
        if not mismatch.file_path.exists():
            return False

        content = self.format_definition_for_file(mismatch.expected_content)
        if not content.strip():
            return False

        mismatch.file_path.write_text(content)
        print(f"  Replaced: {mismatch.file_path.relative_to(self.root)}")
        return True

    def check_message_definitions(self) -> None:
        """Check that message types exist and match exactly."""
        for entry in self.entries:
            if not entry.msg_type:
                continue

            parsed = self.normalize_msg_type(entry.msg_type)
            if not parsed:
                continue

            pkg, typ = parsed

            if not self.check_msg_exists(pkg, typ):
                self.missing.append(f"[{entry.name}] {pkg}/{typ}")
                self.missing_types.append((pkg, typ, entry.interface_type, entry.definition, entry.name))
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

                    actual_set = set(actual_fields)
                    expected_set = set(expected_fields)

                    missing_fields = [f for f in expected_fields if f not in actual_set]
                    extra_fields = [f for f in actual_fields if f not in expected_set]

                    if missing_fields or extra_fields:
                        self.mismatches.append(
                            Mismatch(
                                topic=entry.name,
                                msg_type=typ,
                                file_path=msg_path,
                                interface_type=entry.interface_type,
                                expected_content=entry.definition,
                                actual_content=msg_path.read_text(),
                                missing_fields=missing_fields,
                                extra_fields=extra_fields,
                            )
                        )

    def run(self) -> bool:
        """Run all checks. Returns True if no issues."""
        print(f"Parsing ICD from {self.icd_path}...")
        self.parse_icd()
        print(f"Found {len(self.entries)} ICD entries\n")

        print("Checking message definitions...\n")
        self.check_message_definitions()

        if self.missing:
            print("=" * 60)
            print(f"MISSING ({len(self.missing)}):")
            print("=" * 60)
            for m in self.missing:
                print(f"  {m}")
            print()

        if self.mismatches:
            print("=" * 60)
            print(f"MISMATCH ({len(self.mismatches)}):")
            print("=" * 60)
            for mm in self.mismatches:
                print(f"  [{mm.topic}] {mm.msg_type}")
                for fld in mm.missing_fields:
                    print(f"    - missing: {fld}")
                for fld in mm.extra_fields:
                    print(f"    + extra:   {fld}")
            print()

        if not self.missing and not self.mismatches:
            print("All checks passed!")

        print(f"\nSummary: {len(self.missing)} missing, {len(self.mismatches)} mismatches")

        if self.fix is not None:
            fixed_any = False

            if self.missing_types:
                missing_to_fix = [(p, t, i, d, n) for p, t, i, d, n in self.missing_types if self.should_fix(n)]
                if missing_to_fix:
                    print("\n" + "=" * 60)
                    print("FIXING: Creating missing types...")
                    print("=" * 60)
                    for pkg, typ, iface_type, definition, _ in missing_to_fix:
                        if self.create_missing_type(pkg, typ, iface_type, definition):
                            fixed_any = True

            if self.mismatches:
                mismatches_to_fix = [mm for mm in self.mismatches if self.should_fix(mm.topic)]
                if mismatches_to_fix:
                    print("\n" + "=" * 60)
                    print("FIXING: Replacing mismatched files with ICD spec...")
                    print("=" * 60)
                    for mismatch in mismatches_to_fix:
                        if self.fix_mismatch(mismatch):
                            fixed_any = True

            if fixed_any:
                print("\nFiles modified, rebuild for changes to take effect. ")
            elif self.fix != "":
                print(f"\nNo matching topic found for: {self.fix}")

        return len(self.missing) == 0 and len(self.mismatches) == 0


def main():
    parser = argparse.ArgumentParser(description="Check codebase compliance against ICD specification")
    parser.add_argument("icd_file", type=Path, help="Path to the ICD CSV file")
    parser.add_argument(
        "--fix",
        nargs="?",
        const="",
        default=None,
        metavar="TOPIC",
        help="Fix mismatches. Optionally specify topic (e.g., --fix /arm_thr_cmd)",
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
