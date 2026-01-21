import os
import tkinter as tk
from PIL import Image, ImageTk
from pathlib import Path

IMAGE_DIR = "urc-images-25-26"
OUTPUT_DIR = "crops"
CROP_W = 200
CROP_H = 200

os.makedirs("crops/urc-images-25-26/mallet", exist_ok=True)
os.makedirs("crops/urc-images-25-26/bottle", exist_ok=True)
os.makedirs("crops/urc-images-25-26/pick", exist_ok=True)

class Cropper:
    def __init__(self, root):
        self.root = root
        self.root.title("Image Cropper")

        self.images = [
            f for f in Path(IMAGE_DIR).rglob("*")
            if f.suffix.lower() == ".jpg"
        ]

        self.index = 0

        self.canvas = tk.Canvas(root)
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_click)
        self.load_image()

    def load_image(self):
        img_path = os.path.join(self.images[self.index])
        self.image = Image.open(img_path)

        resize_dimensions = [self.image.width//2, self.image.height//2]
        self.resized_image = self.image.resize(resize_dimensions, Image.Resampling.LANCZOS)
        
        self.tk_image = ImageTk.PhotoImage(self.resized_image)

        self.canvas.config(
            width=self.resized_image.width,
            height=self.resized_image.height
        )
        self.canvas.create_image(0, 0, anchor="nw", image=self.tk_image)

    def on_click(self, event):
        y = event.y*2
        if y < 360:
            y = 360
        elif y > 1560:
            y = 1560

        left = 0
        top = y-360
        right = 1280
        bottom = y+360
        crop = self.image.crop((left, top, right, bottom))
        name = self.images[self.index]
        dest = os.path.join(OUTPUT_DIR, name)
        crop.save(dest)
        print(f"Cropped image stored in {dest}")
        self.index += 1
        if self.index >= len(self.images):
            print("Done!")
            self.root.quit()
            return

        self.canvas.delete("all")
        self.load_image()

root = tk.Tk()
Cropper(root)
root.mainloop()

