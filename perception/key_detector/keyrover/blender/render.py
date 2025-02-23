import bpy
import math
import os
from tqdm import tqdm

import random

randomize_scene = bpy.data.texts["randomize_scene"].as_module()
segmask = bpy.data.texts["segmask"].as_module()

# PARAMETERS CONFIGURATION
# --------------------------------------------

PATH = os.path.dirname(bpy.data.filepath) + "/"
N = 5000

# SCRIPT BEGINS HERE
# --------------------------------------------

scene = bpy.context.scene
scene.render.engine = "CYCLES"

for frame in tqdm(range(1, N)):
    # keyboard = randomize_scene.randomize_scene(seed=frame)
    random.seed(frame)
    randomize_scene.randomize_sun()
    randomize_scene.randomize_camera()
    keyboard = random.choice((1, 1, 2, 2, 3))

    filename = f"keyboard_{frame}_{keyboard}.png"

    # Render Keyboard Mask

    scene.view_settings.view_transform = "Raw"
    scene.view_settings.look = "None"

    bpy.ops.render.render(write_still=True)
    bpy.data.images["Render Result"].save_render(f"{PATH}/texcoords/{filename}")

    # Render Key Mask

    segmask.set_key_mask(keyboard, True)
    bpy.ops.render.render(write_still=True)
    bpy.data.images["Render Result"].save_render(f"{PATH}/masks/{filename}")

    # Render Image

   scene.view_settings.view_transform = "Standard"
   scene.view_settings.look = "Very Low Contrast"
   segmask.set_key_mask(keyboard, False)

   bpy.ops.render.render(write_still=True)
   bpy.data.images["Render Result"].save_render(f"{PATH}/renders/{filename}")
