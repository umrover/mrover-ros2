import bpy
from mathutils import Matrix

from math import radians


def get_child_by_name(collection, name):
    for child in collection.all_objects:
        if child.name.startswith(name):
            return child


def get_children_by_name(collection, name):
    objs = []

    for child in collection.all_objects:
        if child.name.startswith(name):
            if child.data is None:
                continue

            objs.append(child)

    return objs


# See https://blender.stackexchange.com/questions/169672/object-hide-render-crashes-blender-before-starting-to-bake
def get_collection_objects(collection):
    obj_names = [obj.name for obj in collection.all_objects]

    for name in obj_names:
        obj = bpy.data.objects.get(name)
        yield obj


def hide_collection(collection, val=True, exclude=()):
    for obj in get_collection_objects(collection):
        if obj in exclude:
            continue

        obj.hide_render = val
        obj.hide_set(val)


# See https://blender.stackexchange.com/questions/27491/python-vertex-normal-according-to-world
def get_plane_normal(obj):
    normal = obj.data.vertices[0].normal.to_4d()
    normal.w = 0
    normal = (obj.matrix_world @ normal).to_3d()
    return normal / normal.length


def point_at(camera, target, roll=0):
    direction = target - camera.location
    tracker, rotator = (("-Z", "Y"), "Z")
    quat = direction.to_track_quat(*tracker)

    quat = quat.to_matrix().to_4x4()
    roll = radians(roll)
    roll_matrix = Matrix.Rotation(roll, 4, rotator)

    loc = camera.location.to_tuple()
    camera.matrix_world = quat @ roll_matrix
    camera.location = loc

    return direction


def delete_empty_objects():
    objs = bpy.data.objects

    for obj in bpy.context.scene.objects:
        if obj.name.startswith("Empty"):
            objs.remove(objs[obj.name], do_unlink=True)

    bpy.ops.object.delete()


def set_obj_material(obj, index, mat):
    if len(obj.data.materials) == index:
        obj.data.materials.append(mat)

    obj.data.materials[index] = mat
