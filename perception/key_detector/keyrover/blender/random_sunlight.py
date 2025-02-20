import bpy

import math
import random

utils = bpy.data.texts["utils"].as_module()


# TAKEN FROM https://gist.github.com/CGArtPython/149f0274f93f39dedcc17c2364251096

def set_up_world_sun_light(sun_config=None, strength=1.0):
    world_node_tree = bpy.context.scene.world.node_tree
    world_node_tree.nodes.clear()

    node_location_x_step = 300
    node_location_x = 0

    node_sky = world_node_tree.nodes.new(type="ShaderNodeTexSky")
    node_location_x += node_location_x_step

    world_background_node = world_node_tree.nodes.new(type="ShaderNodeBackground")
    world_background_node.inputs["Strength"].default_value = strength
    world_background_node.location.x = node_location_x
    node_location_x += node_location_x_step

    world_output_node = world_node_tree.nodes.new(type="ShaderNodeOutputWorld")
    world_output_node.location.x = node_location_x

    if sun_config:
        print("Updating ShaderNodeTexSky params:")
        for attr, value in sun_config.items():
            if hasattr(node_sky, attr):
                print("  ", attr, "set to", value)
                setattr(node_sky, attr, value)
            else:
                print("\t warning: %s is not an attribute of ShaderNodeTexSky node", attr)

    world_node_tree.links.new(node_sky.outputs["Color"], world_background_node.inputs["Color"])
    world_node_tree.links.new(world_background_node.outputs["Background"], world_output_node.inputs["Surface"])



sun_config = {
    "sun_size": math.radians(random.uniform(0.1, 10)),
    "sun_intensity": random.uniform(0.1, 0.3),
    "air_density": 10 * random.betavariate(2, 4), # random.uniform(1, 10),
    "dust_density": random.uniform(1, 10),
    "sun_elevation": random.uniform(0, 1.5708),
    "sun_rotation": random.uniform(0, 1.5708),
    "ozone_density": random.uniform(1, 10),
    "altitude": random.uniform(0, 60),
}

set_up_world_sun_light(sun_config)
