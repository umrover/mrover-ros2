#!/usr/bin/env python3
"""
cam_mapper.py - watch udev for cameras being plugged in and append
port-based (KERNELS) symlink rules to a udev rules file.

Start with all cameras unplugged, run this, then plug cameras in one at a
time into the port each will live in. For every new camera you're prompted
for a symlink name, and a line like this gets appended:

  SUBSYSTEM=="video4linux", KERNELS=="1-4.1.4.4", ATTR{index}=="0", SYMLINK+="mobcam_left"

Usage:
  sudo ./cam_mapper.py [rules_file]      (default: /etc/udev/rules.d/99-usb-cam.rules)

Ctrl-C to quit. On exit, rules are reloaded and video4linux devices are
re-triggered so the new symlinks show up in /dev right away.
"""
import os
import re
import shutil
import subprocess
import sys

RULES_FILE = sys.argv[1] if len(sys.argv) > 1 else "/etc/udev/rules.d/99-usb-cam.rules"
RULE_FMT = 'SUBSYSTEM=="video4linux", KERNELS=="{kernels}", ATTR{{index}}=="0", SYMLINK+="{name}"\n'

# "UDEV  [158.962365] add      /devices/.../video4linux/video6 (video4linux)"
EVENT_RE = re.compile(r"^UDEV\s+\[[\d.]+\]\s+add\s+(\S+)\s+\(video4linux\)")
# USB device component just before the interface, e.g. ".../5-1.3/5-1.3:1.0/video4linux/video6" -> "5-1.3"
PORT_RE = re.compile(r"/(\d+-[\d.]+):[\d.]+/video4linux/")
NAME_RE = re.compile(r"^[A-Za-z0-9_\-]+$")

_backed_up = False


def read_sysfs(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def load_rules():
    """Return (lines, {kernels: symlink}, {all symlink names})."""
    if not os.path.exists(RULES_FILE):
        return [], {}, set()
    with open(RULES_FILE) as f:
        lines = f.readlines()
    by_port, names = {}, set()
    for ln in lines:
        if ln.lstrip().startswith("#"):
            continue
        k = re.search(r'KERNELS=="([^"]+)"', ln)
        s = re.search(r'SYMLINK\+="([^"]+)"', ln)
        if s:
            names.add(s.group(1))
            if k:
                by_port[k.group(1)] = s.group(1)
    return lines, by_port, names


def write_rule(kernels, name, replace=False):
    global _backed_up
    lines, _, _ = load_rules()

    if replace:
        lines = [
            l for l in lines
            if l.lstrip().startswith("#") or f'KERNELS=="{kernels}"' not in l
        ]

    if lines and not lines[-1].endswith("\n"):
        lines[-1] += "\n"
    lines.append(RULE_FMT.format(kernels=kernels, name=name))

    if not _backed_up and os.path.exists(RULES_FILE):
        shutil.copy2(RULES_FILE, RULES_FILE + ".bak")
        _backed_up = True

    with open(RULES_FILE, "w") as f:
        f.writelines(lines)

    subprocess.run(["udevadm", "control", "--reload-rules"], check=False)


def describe(kernels):
    base = f"/sys/bus/usb/devices/{kernels}"
    vid = read_sysfs(f"{base}/idVendor") or "????"
    pid = read_sysfs(f"{base}/idProduct") or "????"
    mfr = read_sysfs(f"{base}/manufacturer") or ""
    prod = read_sysfs(f"{base}/product") or ""
    label = " ".join(x for x in (mfr, prod) if x) or "unknown device"
    return f"{label} [{vid}:{pid}]"


def ask(prompt):
    return input(prompt).strip()


def handle(kernels, devnode):
    _, by_port, names = load_rules()

    print()
    print(f"New camera on port {kernels}: {describe(kernels)}  ({devnode})")

    replace = False
    if kernels in by_port:
        ans = ask(f"  Port {kernels} is already mapped to '{by_port[kernels]}'. Replace? [y/N] ")
        if ans.lower() != "y":
            print("  Skipped.")
            return
        replace = True
        names.discard(by_port[kernels])

    while True:
        name = ask("  Symlink name (blank to skip): ")
        if not name:
            print("  Skipped.")
            return
        if not NAME_RE.match(name):
            print("  Use only letters, numbers, '_' and '-'.")
            continue
        if name in names:
            ans = ask(f"  '{name}' is already used by another rule. Use it anyway? [y/N] ")
            if ans.lower() != "y":
                continue
        break

    write_rule(kernels, name, replace)
    print(f"  Added: {RULE_FMT.format(kernels=kernels, name=name).strip()}")


def main():
    rules_dir = os.path.dirname(os.path.abspath(RULES_FILE))
    target = RULES_FILE if os.path.exists(RULES_FILE) else rules_dir
    if not os.access(target, os.W_OK):
        sys.exit(f"Can't write {RULES_FILE} - run with sudo.")

    cmd = ["udevadm", "monitor", "--udev", "--subsystem-match=video4linux"]
    if shutil.which("stdbuf"):
        cmd = ["stdbuf", "-oL"] + cmd  # make sure events arrive line by line

    print(f"Writing rules to {RULES_FILE}")
    print("Plug in cameras one at a time. Ctrl-C to finish.")

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1)
    added_any = False
    try:
        for line in proc.stdout:
            m = EVENT_RE.match(line)
            if not m:
                continue
            devpath = m.group(1)

            # Each camera usually creates two nodes (index 0 = capture, 1 = metadata);
            # only act on index 0.
            if read_sysfs(f"/sys{devpath}/index") != "0":
                continue

            p = PORT_RE.search(devpath)
            if not p:
                print(f"\nIgnoring non-USB video device: {devpath}")
                continue

            handle(p.group(1), "/dev/" + devpath.rsplit("/", 1)[-1])
            added_any = True
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        proc.terminate()
        if added_any:
            print("Reloading rules and re-triggering video4linux devices...")
            subprocess.run(["udevadm", "control", "--reload-rules"], check=False)
            subprocess.run(["udevadm", "trigger", "--subsystem-match=video4linux"], check=False)
        print("Done.")


if __name__ == "__main__":
    main()
