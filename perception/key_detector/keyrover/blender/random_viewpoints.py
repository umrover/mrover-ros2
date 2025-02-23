import bpy
from mathutils import Vector
from random import uniform
from numpy import random
from math import acos, degrees

math_utils = bpy.data.texts["math_utils"].as_module()
blender_utils = bpy.data.texts["blender_utils"].as_module()

# PARAMETERS CONFIGURATION
# --------------------------------------------

theta_lims = (-40, 20)  # degrees (0-360), x-axis
theta_std = 0.3

phi_lims = (65, 115)  # degrees (0-180), y-axis
phi_std = 0.3

rho_lims = (2, 5)
max_roll = 15

keyboard = 4

N = 1000

TOTAL_KEYBOARDS = 4

LOCATION_SAMPLING = "gaussian"
THETA_SAMPLING = "gaussian"
PHI_SAMPLING = "gaussian"

# SCRIPT BEGINS HERE
# --------------------------------------------

for i in range(TOTAL_KEYBOARDS):
    objs = bpy.data.collections[f"Keyboard {i + 1}"]
    blender_utils.hide_collection(objs, True)

keyboard = bpy.data.collections[f"Keyboard {keyboard}"]
keyboard_plane = blender_utils.get_child_by_name(keyboard, "Orientation")
camera = bpy.data.objects["Camera"]

blender_utils.hide_collection(keyboard, False, exclude=(keyboard_plane,))
blender_utils.delete_empty_objects()

# Adjusting theta lims to account for orientation vector

normal = blender_utils.get_plane_normal(keyboard_plane)
angle = math_utils.angle_between(normal, Vector((0, 0, 1)))
theta_lims = (theta_lims[0] - angle, theta_lims[1] - angle)

bpy.context.scene.frame_start = 1
bpy.context.scene.frame_end = N


def get_random_angle():
    return math_utils.get_random_arc_angle(*theta_lims, *phi_lims,
                                           THETA_SAMPLING, PHI_SAMPLING,
                                           theta_std, phi_std)


for i in range(N):
    random_angle = get_random_angle()
    random_dist = uniform(*rho_lims)
    random_vec = math_utils.spherical_to_cartesian(random_dist, *random_angle)
    random_vec = Vector(random_vec)

    if LOCATION_SAMPLING == "gaussian":
        random_x = max(-1, min(1, random.normal(0, 0.5)))
        random_y = max(-1, min(1, random.normal(0, 0.5)))

    elif LOCATION_SAMPLING == "uniform":
        random_x = uniform(-1, 1)
        random_y = uniform(-1, 1)

    else:
        raise ValueError("Unknown Location Sampling")

    random_translation = Vector((random_x, random_y, 0))
    random_translation = keyboard_plane.matrix_world @ random_translation
    random_vec += random_translation

    random_roll = uniform(-max_roll, max_roll)

    # Create Camera Keyframe

    camera.location = random_vec
    blender_utils.point_at(camera, random_translation, random_roll)

    camera.keyframe_insert(data_path="location", frame=i)
    camera.keyframe_insert(data_path="rotation_euler", frame=i)

    if uniform(0, 1) > 100 / N:
        continue

    # Add line showing angle

    mesh = bpy.data.meshes.new("Empty")
    obj = bpy.data.objects.new("Empty", mesh)
    keyboard.objects.link(obj)

    mesh.from_pydata([random_translation, random_vec], [(0, 1)], [])
    mesh.update()
