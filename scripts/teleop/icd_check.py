#!/usr/bin/env python3
"""
ICD Compliance Checker - validates message definitions against ICD spec.

Usage:
    icd_check.py <icd.csv>                  Check compliance
    icd_check.py <icd.csv> --fix            Fix all mismatches
    icd_check.py <icd.csv> --fix /topic     Fix specific topic only
"""

import argparse
import csv
import re
import sys
from dataclasses import dataclass
from pathlib import Path

IFACE_DIRS = {"Topic": ("msg", ".msg"), "Service": ("srv", ".srv"), "Action": ("action", ".action")}


@dataclass
class Entry:
    name: str
    interface_type: str
    pkg: str
    typ: str
    definition: str


def normalize_field(line: str) -> str | None:
    line = line.split("#")[0].strip()
    if not line or line.endswith(":"):
        return None
    if line == "---":
        return "---"
    line = re.sub(r"\[\d*\]", "[]", line)
    parts = line.split()
    if len(parts) == 1 and "/" in parts[0]:
        line = f"{parts[0]} {parts[0].split('/')[-1].lower()}"
    return line


def normalize_definition(definition: str) -> str:
    fields = []
    for line in definition.split("\n"):
        if not line.strip():
            break
        if f := normalize_field(line):
            fields.append(f)
    return "\n".join(fields) + "\n" if fields else ""


def parse_icd(path: Path) -> list[Entry]:
    rows = list(csv.reader(open(path)))
    header_idx = next((i for i, r in enumerate(rows) if r and r[0] == "Subsystem"), None)
    if header_idx is None:
        return []

    entries = []
    for row in rows[header_idx + 1 :]:
        if len(row) < 7 or not row[0].strip():
            continue
        name = row[4].strip()
        if not name or name.startswith("/delta"):
            continue
        msg_type = row[6].strip().replace("gemetry_msgs", "geometry_msgs")
        if "/" not in msg_type:
            continue
        parts = msg_type.split("/")
        entries.append(Entry(name, row[3].strip(), parts[0], parts[-1], row[7].strip() if len(row) > 7 else ""))
    return entries


def get_file_path(root: Path, entry: Entry) -> Path | None:
    if entry.pkg != "mrover" or entry.interface_type not in IFACE_DIRS:
        return None
    dir_name, ext = IFACE_DIRS[entry.interface_type]
    return root / dir_name / f"{entry.typ}{ext}"


def check_and_fix(root: Path, entries: list[Entry], fix_topic: str | None) -> bool:
    missing, mismatches = [], []

    for entry in entries:
        path = get_file_path(root, entry)
        if not path:
            continue

        expected = normalize_definition(entry.definition)
        if not path.exists():
            missing.append((entry, path, expected))
        elif entry.definition and path.read_text() != expected:
            mismatches.append((entry, path, expected))

    if missing:
        print(f"MISSING ({len(missing)}):")
        for entry, path, _ in missing:
            print(f"  [{entry.name}] {entry.pkg}/{entry.typ}")

    if mismatches:
        print(f"MISMATCH ({len(mismatches)}):")
        for entry, path, expected in mismatches:
            actual_fields = {f for l in path.read_text().split("\n") if (f := normalize_field(l))}
            expected_fields = set(expected.split("\n")) - {""}
            missing_f = expected_fields - actual_fields
            extra_f = actual_fields - expected_fields
            if missing_f or extra_f:
                for f in missing_f:
                    print(f"  [{entry.name}] - {f}")
                for f in extra_f:
                    print(f"  [{entry.name}] + {f}")
            else:
                print(f"  [{entry.name}] {path.name} (formatting)")

    if not missing and not mismatches:
        print("All checks passed!")
        return True

    print(f"\nSummary: {len(missing)} missing, {len(mismatches)} mismatches")

    if fix_topic is not None:
        should_fix = (lambda e: True) if fix_topic == "" else (lambda e: e.name == fix_topic)
        fixed = 0
        for entry, path, expected in missing + mismatches:
            if should_fix(entry) and expected:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(expected)
                print(f"  Fixed: {path.relative_to(root)}")
                fixed += 1
        if fixed:
            print("\nFiles modified, rebuild for changes to take effect.")
        elif fix_topic:
            print(f"\nNo matching topic: {fix_topic}")

    return False


def main():
    parser = argparse.ArgumentParser(description="Check ICD compliance")
    parser.add_argument("icd_file", type=Path)
    parser.add_argument("--fix", nargs="?", const="", default=None, metavar="TOPIC")
    args = parser.parse_args()

    if not args.icd_file.exists():
        sys.exit(f"ICD file not found: {args.icd_file}")

    root = Path(__file__).parent.parent.parent
    entries = parse_icd(args.icd_file)
    print(f"Found {len(entries)} ICD entries\n")
    sys.exit(0 if check_and_fix(root, entries, args.fix) else 1)


if __name__ == "__main__":
    main()
