import bpy
import os
from tqdm import tqdm

import random
import pickle

randomize_scene = bpy.data.texts["randomize_scene"].as_module()
segmask = bpy.data.texts["segmask"].as_module()

# PARAMETERS CONFIGURATION
# --------------------------------------------

PATH = os.path.dirname(bpy.data.filepath) + "/"
N = 5000

# SCRIPT BEGINS HERE
# --------------------------------------------

camera = bpy.context.scene.camera

camera_data = {"location": [],
               "rotation": []}

for frame in tqdm(range(1, N)):
    random.seed(frame)
    randomize_scene.randomize_sun()
    randomize_scene.randomize_camera()
    keyboard = random.choice((1, 1, 2, 2, 3))

    filename = f"keyboard_{frame}_{keyboard}.png"

    camera_data["location"].append(tuple(camera.location))
    camera_data["location"].append(tuple(camera.rotation_euler))

with open(f"{PATH}/camera_data.bin", "wb") as file:
    pickle.dump(camera_data, file)
