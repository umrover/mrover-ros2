#!/usr/bin/env python3
"""Generate simulator/models/ from the URDF/xacro + FBX sources.

Outputs: 
- mjcf/*.xml
- collision *.obj
- visual *.msh
- textures

URDF->MJCF and FBX->.msh run via the model_tools binary.
This env reaches MuJoCo/assimp only through their C/C++ libs.
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import TypedDict, cast

REPO = Path(__file__).resolve().parent.parent
MUJOCO = REPO / "deps" / "mujoco-prebuilt"
MESHES = REPO / "simulator" / "models" / "meshes"
MJCF = REPO / "simulator" / "models" / "mjcf"
VISUAL_DIR = MESHES / "visual"
MESH_SRC = REPO / "urdf" / "meshes"
TEXTURES_SRC = REPO / "urdf" / "textures"
TEXTURES_OUT = REPO / "simulator" / "models" / "textures"
TOOL = REPO / "build" / "model_tools"

# stem -> source xacro (world.urdf is intentionally omitted; the scene uses a plane).
MODELS = {
    "rover": "urdf/rover/rover.urdf.xacro",
    "rock": "urdf/world/rock.urdf.xacro",
    "tag_0": "urdf/world/tag_0.urdf.xacro",
    "tag_1": "urdf/world/tag_1.urdf.xacro",
    "hammer": "urdf/world/hammer.urdf.xacro",
    "bottle": "urdf/world/bottle.urdf.xacro",
    "lander": "urdf/world/lander.urdf.xacro",
    "mars": "urdf/world/mars.urdf.xacro",
}

VISUAL_GROUP = "2"
COLLISION_GROUP = "3"
FBX_RE = re.compile(r"package://mrover/urdf/meshes/(.+\.fbx)$")


class MaterialInfo(TypedDict):
    rgba: list[float]
    texture: str | None
    file: str


class VisualEntry(TypedDict):
    stem: str
    pos: list[float]
    quat: tuple[float, float, float, float]


def sanitize(name: str) -> str:
    return re.sub(r"[^0-9A-Za-z]+", "_", name).strip("_")


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return w, x, y, z


def build_tool() -> None:
    TOOL.parent.mkdir(parents=True, exist_ok=True)
    print("Building model_tools...")
    pkgcfg = subprocess.check_output(["pkg-config", "--cflags", "--libs", "assimp"]).split()
    _ = subprocess.run(
        ["g++", "-std=c++20", "-O2", str(REPO / "scripts" / "model_tools.cpp"),
         "-I", str(MUJOCO / "include"), "-L", str(MUJOCO / "lib"), "-lmujoco",
         *[a.decode() for a in pkgcfg], "-o", str(TOOL)],
        check=True,
    )


def xacro(src: Path) -> ET.Element:
    out = subprocess.run(["xacro", str(src)], check=True, capture_output=True, text=True)
    return ET.fromstring(out.stdout)


def export_collision_meshes(urdf_root: ET.Element) -> None:
    """Export each <collision> FBX to a welded OBJ (MuJoCo's collision convex-hull source)."""
    seen: set[str] = set()
    for link in urdf_root.iter("link"):
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            m = FBX_RE.search(mesh.get("filename", ""))
            if not m or m.group(1) in seen:
                continue
            seen.add(m.group(1))
            rel = m.group(1)
            src = MESH_SRC / rel
            obj = MESHES / f"{rel[:-4]}.obj"
            if obj.exists() and obj.stat().st_mtime >= src.stat().st_mtime:
                continue
            print(f"  mesh {rel} -> {obj.name}")
            obj.parent.mkdir(parents=True, exist_ok=True)
            _ = subprocess.run(["assimp", "export", str(src), str(obj), "-jiv"],
                                 check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            obj.with_suffix(".mtl").unlink(missing_ok=True)  # MuJoCo ignores .mtl


def prepare_urdf(urdf_root: ET.Element, stem: str) -> ET.Element:
    """Make the URDF MuJoCo-compilable: give massless links a tiny inertial, drop
    off-diagonal inertia terms, point meshes at the exported OBJs (cm->m for props),
    and prepend a <mujoco> compiler block (discard FBX visuals, bound masses/inertias)."""
    for link in urdf_root.iter("link"):
        if len(link) == 0:
            inertial = ET.SubElement(link, "inertial")
            _ = ET.SubElement(inertial, "mass", {"value": "0.001"})
            _ = ET.SubElement(inertial, "inertia", {
                "ixx": "1e-5", "ixy": "0", "ixz": "0", "iyy": "1e-5", "iyz": "0", "izz": "1e-5"})

    for inertia in urdf_root.iter("inertia"):
        for off in ("ixy", "ixz", "iyz"):
            inertia.set(off, "0")

    for mesh in urdf_root.iter("mesh"):
        m = FBX_RE.search(mesh.get("filename", ""))
        if m:
            mesh.set("filename", f"{m.group(1)[:-4]}.obj")
        if stem != "rover":  # world props are authored in centimeters
            mesh.set("scale", "0.01 0.01 0.01")

    mujoco = ET.Element("mujoco")
    _ = ET.SubElement(mujoco, "compiler", {
        "meshdir": str(MESHES), "discardvisual": "true", "fusestatic": "false",
        "balanceinertia": "true", "boundmass": "0.001", "boundinertia": "1e-5"})
    urdf_root.insert(0, mujoco)
    return urdf_root


def convert_texture(src: Path) -> str | None:
    """Copy/convert a texture into the output dir as PNG (MuJoCo only loads PNG),
    downscaling large images. Returns the filename, or None if the source is missing."""
    if not src.exists():
        return None
    TEXTURES_OUT.mkdir(parents=True, exist_ok=True)
    if src.suffix.lower() in (".jpg", ".jpeg"):
        dst = TEXTURES_OUT / f"{src.stem}.png"
        if not dst.exists():
            _ = subprocess.run(["magick", str(src), "-resize", "2048x2048>", str(dst)],
                                 check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return dst.name
    dst = TEXTURES_OUT / src.name
    if not dst.exists():
        _ = shutil.copy2(src, dst)
    return dst.name


def process_fbx(stem: str) -> dict[str, MaterialInfo]:
    """Split one FBX into per-material .msh via model_tools, returning the material map
    (referenced textures converted to PNG for MuJoCo)."""
    out_dir = VISUAL_DIR / stem
    fbx = MESH_SRC / f"{stem}.fbx"
    if not fbx.exists():
        print(f"  missing FBX {fbx}", file=sys.stderr)
        return {}
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    _ = subprocess.run([str(TOOL), "fbx2msh", str(fbx), str(out_dir)], check=True)

    mats = cast(dict[str, MaterialInfo], json.loads((out_dir / "materials.json").read_text()))
    for info in mats.values():
        if info["texture"]:
            info["texture"] = convert_texture(TEXTURES_SRC / info["texture"])
    return mats


def link_visuals(urdf_root: ET.Element) -> dict[str, list[VisualEntry]]:
    """link name -> list of {stem, pos, quat} from the URDF <visual> meshes."""
    out: dict[str, list[VisualEntry]] = {}
    for link in urdf_root.iter("link"):
        name = link.get("name")
        if not name:
            continue
        entries: list[VisualEntry] = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            m = re.search(r"/([^/]+)\.fbx$", mesh.get("filename", ""))
            if not m:
                continue
            origin = visual.find("origin")
            xyz_str = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
            rpy_str = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
            xyz = [float(v) for v in xyz_str.split()]
            rpy = [float(v) for v in rpy_str.split()]
            entries.append({"stem": m.group(1), "pos": xyz, "quat": rpy_to_quat(*rpy)})
        if entries:
            out[name] = entries
    return out


def find_body(parent: ET.Element, name: str) -> ET.Element | None:
    for body in parent.iter("body"):
        if body.get("name") == name:
            return body
    return None


def inject_visuals(expanded_root: ET.Element, mjcf_path: Path, scale: float) -> None:
    """Add one non-colliding visual geom (group 2) per material to each body, pushing
    the existing primitive collision geoms to group 3 so the viewer shows the meshes."""
    scale_attr = f"{scale} {scale} {scale}"
    visuals = link_visuals(expanded_root)

    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("texturedir", "../textures")

    asset = root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        root.insert(list(root).index(compiler) + 1, asset)

    for geom in root.iter("geom"):
        geom.set("group", COLLISION_GROUP)

    used_stems: dict[str, dict[str, MaterialInfo]] = {}
    declared_tex: set[str] = set()
    declared_assets: set[str] = set()

    for link, entries in visuals.items():
        body = find_body(root, link)
        if body is None:
            continue
        for entry in entries:
            stem = entry["stem"]
            if stem not in used_stems:
                used_stems[stem] = process_fbx(stem)
            for matsan, info in used_stems[stem].items():
                asset_id = f"{stem}__{matsan}"
                if asset_id not in declared_assets:
                    declared_assets.add(asset_id)
                    tex = info["texture"]
                    if tex:
                        texname = f"tex_{sanitize(Path(tex).stem)}"
                        if texname not in declared_tex:
                            declared_tex.add(texname)
                            _ = ET.SubElement(asset, "texture", {"name": texname, "type": "2d", "file": tex})
                        _ = ET.SubElement(asset, "material", {
                            "name": asset_id, "texture": texname, "specular": "0.3", "shininess": "0.3"})
                    else:
                        rgba = " ".join(f"{c:.4f}" for c in info["rgba"])
                        _ = ET.SubElement(asset, "material", {
                            "name": asset_id, "rgba": rgba, "specular": "0.3", "shininess": "0.3"})
                    # shell inertia: avoids "volume too small" errors on flat decals.
                    _ = ET.SubElement(asset, "mesh", {
                        "name": asset_id, "file": f"visual/{stem}/{info['file']}",
                        "scale": scale_attr, "inertia": "shell"})

                w, x, y, z = entry["quat"]
                px, py, pz = entry["pos"]
                _ = ET.SubElement(body, "geom", {
                    "type": "mesh", "mesh": asset_id, "material": asset_id,
                    "contype": "0", "conaffinity": "0", "group": VISUAL_GROUP,
                    "pos": f"{px} {py} {pz}", "quat": f"{w} {x} {y} {z}"})

    ET.indent(tree, space="  ")
    tree.write(mjcf_path, encoding="unicode")
    print(f"  {mjcf_path.name}: {len(declared_assets)} visual sub-meshes across {len(visuals)} links")


def main() -> int:
    if not (MUJOCO / "lib").is_dir():
        print(f"MuJoCo not found at {MUJOCO}. Run scripts/setup_mujoco.sh first.", file=sys.stderr)
        return 1
    libs = [str(MUJOCO / "lib")]
    if conda := os.environ.get("CONDA_PREFIX"):
        libs.append(str(Path(conda) / "lib"))
    if existing := os.environ.get("LD_LIBRARY_PATH"):
        libs.append(existing)
    os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(libs)

    MESHES.mkdir(parents=True, exist_ok=True)
    MJCF.mkdir(parents=True, exist_ok=True)
    build_tool()

    for stem, rel in MODELS.items():
        expanded = xacro(REPO / rel)
        export_collision_meshes(expanded)

        prepared = prepare_urdf(ET.fromstring(ET.tostring(expanded)), stem)
        prepared_path = MJCF / f"{stem}.prepared.urdf"
        ET.ElementTree(prepared).write(prepared_path, encoding="unicode")

        mjcf = MJCF / f"{stem}.xml"
        _ = subprocess.run([str(TOOL), "urdf2mjcf", str(prepared_path), str(mjcf)], check=True)
        prepared_path.unlink()

        # Make the committed model portable: meshes resolve relative to the mjcf dir.
        _ = mjcf.write_text(re.sub(rf'meshdir="{re.escape(str(MESHES))}/?"', 'meshdir="../meshes"', mjcf.read_text()))

        inject_visuals(expanded, mjcf, scale=1.0 if stem == "rover" else 0.01)

    # materials.json is just the model_tools -> inject_visuals handoff; nothing reads it
    # at runtime, so don't leave it scattered through the committed mesh tree.
    for leftover in VISUAL_DIR.glob("*/materials.json"):
        leftover.unlink()

    print(f"Done. MJCF in {MJCF.relative_to(REPO)}, meshes in {MESHES.relative_to(REPO)}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
