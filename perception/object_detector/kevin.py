import tkinter
from PIL import ImageTk, Image, ImageOps
from pathlib import Path
import os

image_dir = Path("urc-images-25-26")

for path in image_dir.rglob("*"):
    if path.suffix.lower() == ".jpg":
        print(f"Rescaling: {path}")
        img = Image.open(path)
        img = ImageOps.exif_transpose(img)
        
        scale = 1280 / img.width
        new_size = (1280, int(img.height * scale))

        scaled = img.resize(new_size, Image.Resampling.LANCZOS)

        scaled.save(path)
        img.close()

'''
# creating main window
root = tkinter.Tk()

# loading the image
img = ImageTk.PhotoImage(Image.open("images.png"))

# reading the image
panel = tkinter.Label(root, image = img)

# setting the application
panel.pack(side = "bottom", fill = "both",
           expand = "yes")

# running the application
root.mainloop()
'''
