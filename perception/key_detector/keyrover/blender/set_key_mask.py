import bpy
from mathutils import Vector

math_utils = bpy.data.texts["math_utils"].as_module()
blender_utils = bpy.data.texts["blender_utils"].as_module()

# PARAMETERS CONFIGURATION
# --------------------------------------------


SET_MASK = True
KEYBOARD_MASK = True

KEYBOARD = 2

TOTAL_KEYBOARDS = 3


# SCRIPT BEGINS HERE
# --------------------------------------------


def get_ith_color(i):
    r = i % 12
    g = (i // 12) % 12
    b = (i // 144) % 12
    return r * 21, b * 21, g * 21


def create_mask_material(i):
    color = get_ith_color(i)
    name = f"ImageMask - {color}"

    if (mat := bpy.data.materials.get(name)) is None:
        mat = bpy.data.materials.new(name=name)

    # Set Material's Emission

    mat.use_nodes = True
    tree = mat.node_tree
    nodes = tree.nodes

    for node in nodes:
        if node.bl_idname not in {"ShaderNodeBsdfPrincipled", "ShaderNodeOutputMaterial"}:
            nodes.remove(node)

    color = map(lambda c: c / 255, color)

    emit = nodes.new("ShaderNodeEmission")
    emit.inputs["Color"].default_value = (*color, 1)
    emit.inputs["Color"].keyframe_insert("default_value", frame=1)
    new_link = tree.links.new(nodes['Material Output'].inputs['Surface'], nodes['Emission'].outputs['Emission'])

    return mat


def set_key_mask(i, set_mask, hide=False):
    keyboard = bpy.data.collections[f"Keyboard {i}"]
    keys = blender_utils.get_children_by_name(keyboard, "Key")
    keys += blender_utils.get_children_by_name(keyboard, "Numpad")

    orientation = bpy.data.objects["Orientation"]
    orientation = blender_utils.get_plane_normal(orientation)

    if hide:
        blender_utils.hide_collection(keyboard, True)
    else:
        blender_utils.hide_collection(keyboard, False)
        blender_utils.hide_collection(keyboard, set_mask, exclude=keys)

    for i, ob in enumerate(keys):
        assert i <= 255

        mat = create_mask_material(i + 1)

        blender_utils.set_obj_material(ob, 1, black)
        blender_utils.set_obj_material(ob, 2, mat)

        for face in ob.data.polygons:
            if not set_mask:
                face.material_index = 0
                continue

            if abs(face.normal.dot(orientation) - 1) < 0.1:
                face.material_index = 2
            else:
                face.material_index = 1


def set_keyboard_mask(i, set_mask, hide=False):
    keyboard = bpy.data.collections[f"Keyboard {i}"]

    blender_utils.hide_collection(keyboard, hide)
    material = bpy.data.materials.get("Mask")

    for ob in keyboard.all_objects:
        if not hasattr(ob.data, "polygons"):
            continue

        for face in ob.data.polygons:
            if not set_mask:
                face.material_index = 0
                continue

            blender_utils.set_obj_material(ob, 4, material)
            face.material_index = 3


black = create_mask_material(0)

if __name__ == "__main__":
    for i in range(1, TOTAL_KEYBOARDS + 1):
        if KEYBOARD_MASK:
            set_keyboard_mask(i, SET_MASK, i != KEYBOARD)
        else:
            set_key_mask(i, SET_MASK, i != KEYBOARD)
