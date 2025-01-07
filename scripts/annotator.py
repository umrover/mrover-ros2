# Additional Non-MRover Deps
# ultralytics
# PyQt6
# opencv

import sys
import os
import numpy as np
from pathlib import Path
from enum import Enum
from collections import namedtuple
import yaml
import shutil

# QT6 (QT5 is deprecated with opencv)
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton, QFileDialog
from PyQt6.QtGui import QIcon, QPixmap, QCursor, QImage
from PyQt6.QtCore import QSize, Qt

# CV
import cv2

# Ultralytics
from ultralytics import FastSAM

# WINDOW CONSTANTS
STARTING_X_LOCATION = 200//2
STARTING_Y_LOCATION = 100//2

APP_WINDOW_WIDTH = 1920//2
APP_WINDOW_HEIGHT = 1080//2

# LAYOUT CONSTANTS
BUTTON_HEIGHT = 75//2
BUTTON_WIDTH = APP_WINDOW_WIDTH // 3

# MISC.
IMAGE_PATH = 'data/images/pic.png'
IMAGE_RESIZE_FACTOR = 3

# Object Constants
NUM_CLASSES = 2

OBJECT_COLORS = np.array([(247,5,41),
                          (5,118,247)], dtype=np.uint8)

CLASS_NAMES = ['WATER BOTTLE',
               'MALLET']

class ObjectsIdentifier(Enum):
    WATER_BOTTLE = 0
    MALLET = 1

class SelectionMode(Enum):
    MANUAL_SEQUENTIAL = 1
    MANUAL_CLOSEST = 2
    AI = 3
    OBJECT_SELECTION = 4

Object = namedtuple('Object', ['identifier', 'pts'])

def cvmat_to_qpixmap(cvmat):
    height, width, channel = cvmat.shape
    bytesPerLine = channel * width
    qimg = QImage(cvmat.data, width, height, bytesPerLine, QImage.Format.Format_RGB888).rgbSwapped()
    return QPixmap(qimg)

# Ray Casting Algorithm

_eps = 0.00001
_huge = sys.float_info.max
_tiny = sys.float_info.min

def rayintersectseg(p, a, b):
    if a[1] > b[1]:
        a,b = b,a
    if p[1] == a[1] or p[1] == b[1]:
        p = (p[0], p[1] + _eps)

    intersect = False

    if (p[1] > b[1] or p[1] < a[1]) or (
        p[0] > max(a[0], b[0])):
        return False

    if p[0] < min(a[0], b[0]):
        intersect = True
    else:
        if abs(a[0] - b[0]) > _tiny:
            m_red = (b[1] - a[1]) / float(b[0] - a[0])
        else:
            m_red = _huge
        if abs(a[0] - p[0]) > _tiny:
            m_blue = (p[1] - a[1]) / float(p[0] - a[0])
        else:
            m_blue = _huge
        intersect = m_blue >= m_red
    return intersect

class ApplicationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setGeometry(STARTING_X_LOCATION, STARTING_Y_LOCATION, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT)
        # TODO: Make the app resizable
        self.setFixedSize(APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT)

        # Init Selection
        self.objects = []
        self.current_identifier = ObjectsIdentifier.WATER_BOTTLE
        self.current_selection = 0

        # Current Yolo Project
        self.config_path = Path("")

        # Init Buttons
        self.init_buttons()

        # Init Image Viewer
        self.img_path = Path(IMAGE_PATH)
        self.init_image_viewer()

        # Init Mode
        self.mode = SelectionMode.MANUAL_SEQUENTIAL

        # Init AI Model
        self.model = FastSAM("FastSAM-s.pt")

    def _set_image_viewer(self, cvmat):
        self.cvmat = cvmat
        self.image_viewer_pixmap = cvmat_to_qpixmap(self.cvmat)
        self.image_viewer_label.setPixmap(self.image_viewer_pixmap)

    def _render_selection(self):
        overlay = self.cvmat_unedited.copy()

        # Draw all of the masks
        for i in range(len(self.objects)):
            if len(self.objects) != 0:
                pts = self.objects[i].pts.reshape((-1, 1, 2))
                color = tuple(map(int, OBJECT_COLORS[int(self.objects[i].identifier.value)]))
                if pts.size != 0:
                    cv2.fillPoly(overlay, [pts], color)

        # Draw all of the points on the current selection
        if len(self.objects) != 0:
            if self.objects[self.current_selection].pts.size != 0:
                for (x, y) in self.objects[self.current_selection].pts:
                    cv2.circle(overlay, (x, y), 3, (96, 68, 207), -1)

        alpha = 0.5
        result = cv2.addWeighted(overlay, alpha, self.cvmat_unedited, 1 - alpha, 0)
        self._set_image_viewer(result)
        self.top_right.setText(f"Class: {CLASS_NAMES[self.current_identifier.value]}")

    def get_cursor_x(self):
        return self.mapFromGlobal(QCursor.pos()).x()

    def get_cursor_y(self):
        return self.mapFromGlobal(QCursor.pos()).y() - BUTTON_HEIGHT

    def init_buttons(self):
        # TOP LEFT
        self.top_left = QPushButton("Open Image", self)
        self.top_left.setGeometry(0, 0, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.top_left.clicked.connect(self.top_left_click)

        # TOP CENTER
        self.top_center_left = QPushButton("Open Config", self)
        self.top_center_left.setGeometry(BUTTON_WIDTH, 0, BUTTON_WIDTH//2, BUTTON_HEIGHT)
        self.top_center_left.clicked.connect(self.top_center_left_click)

        self.top_center_right = QPushButton("Save Annotation", self)
        self.top_center_right.setGeometry(BUTTON_WIDTH + BUTTON_WIDTH//2, 0, BUTTON_WIDTH//2, BUTTON_HEIGHT)
        self.top_center_right.clicked.connect(self.top_center_right_click)
        self.top_center_right.setEnabled(False)

        # TOP RIGHT
        self.top_right = QPushButton(f"Class: {CLASS_NAMES[self.current_identifier.value]}", self)
        self.top_right.setGeometry(2 * BUTTON_WIDTH, 0, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.top_right.clicked.connect(self.top_right_click)

        # BOTTOM LEFT
        self.bottom_left = QPushButton("Mode: Manual Sequential", self)
        self.bottom_left.setGeometry(0, self.height() - BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.bottom_left.clicked.connect(self.bottom_left_click)

        # BOTTOM CENTER
        self.bottom_center = QPushButton("Clear Selection", self)
        self.bottom_center.setGeometry(BUTTON_WIDTH, self.height() - BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.bottom_center.clicked.connect(self.bottom_center_click)

        # BOTTOM RIGHT
        self.bottom_right = QPushButton("New Object", self)
        self.bottom_right.setGeometry(2 * BUTTON_WIDTH, self.height() - BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.bottom_right.clicked.connect(self.bottom_right_click)

    def init_image_viewer(self):
        self.image_viewer_button = QPushButton(self)
        self.image_viewer_button.setGeometry(0, BUTTON_HEIGHT, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT - 2 * BUTTON_HEIGHT)
        self.image_viewer_button.clicked.connect(self.image_viewer_click)

        self.image_viewer_label = QLabel(self)
        self.image_viewer_label.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents) # allows the click to hit the button instead of the label
        self.image_viewer_label.setGeometry(0, BUTTON_HEIGHT, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT - 2 * BUTTON_HEIGHT)

        self.cvmat_unedited = cv2.imread(self.img_path.__str__())

        self.X_COEFF = self.cvmat_unedited.shape[1]/(APP_WINDOW_WIDTH)
        self.Y_COEFF = self.cvmat_unedited.shape[0]/(APP_WINDOW_HEIGHT - 2 * BUTTON_HEIGHT)

        self._render_selection()
        self.image_viewer_label.setScaledContents(True)

    def top_left_click(self):
        print("Open Image Clicked")
        file_dialog = QFileDialog(self)
        file_dialog.setWindowTitle("Select an image to annotate...")
        file_dialog.setDirectory(Path.cwd().__str__())

        if file_dialog.exec():
            self.img_path = Path(file_dialog.selectedFiles()[0])
            print(f'Selected {self.img_path}')

        self.cvmat_unedited = cv2.imread(self.img_path.__str__())

        self.X_COEFF = self.cvmat_unedited.shape[1]/(APP_WINDOW_WIDTH)
        self.Y_COEFF = self.cvmat_unedited.shape[0]/(APP_WINDOW_HEIGHT - 2 * BUTTON_HEIGHT)

        self.objects = []
        self._render_selection()
        self.current_selection = 0

        self._render_selection()

    def top_center_left_click(self):
        print("Open Config File")
        file_dialog = QFileDialog(self)
        file_dialog.setWindowTitle("Select a yaml file to augment...")
        file_dialog.setDirectory(Path.cwd().__str__())

        if file_dialog.exec():
            self.config_path = Path(file_dialog.selectedFiles()[0])
            print(f'Selected {self.config_path}')

        if self.config_path.exists() and not os.path.isdir(self.config_path):
            self.top_center_right.setEnabled(True)

    def top_center_right_click(self):
        print("Save Annotation")
        with open(self.config_path, 'r') as f:
            data = yaml.load(f, Loader=yaml.SafeLoader)
            training_dataset_dir = Path(data['path'])
            training_dataset_images = training_dataset_dir / Path(data['train'])
            training_dataset_labels = training_dataset_dir / Path(data['train'].replace('images', 'labels'))

            # Copy the image to the training
            shutil.copyfile(self.img_path, training_dataset_images / self.img_path.name, follow_symlinks = True)

            with open(training_dataset_labels / self.img_path.with_suffix('.txt').name, "w") as f:
                for object_index in range(len(self.objects)):
                    # Make sure there are enough points to create a polygon = 3
                    if len(self.objects[object_index].pts) < 3:
                        continue;

                    if self.objects[object_index].pts.size != 0:
                        x1, y1, x2, y2 =np.inf, np.inf, 0, 0 
                        for (x, y) in self.objects[object_index].pts:
                            x1 = min(x, x1)
                            y1 = min(y, y1)
                            x2 = max(x, x2)
                            y2 = max(y, y2)

                        x = (x1 + (x2 - x1) / 2) / self.cvmat_unedited.shape[1]
                        y = (y1 + (y2 - y1) / 2) / self.cvmat_unedited.shape[0]
                        w = (x2 - x1) / self.cvmat_unedited.shape[1]
                        h = (y2 - y1) / self.cvmat_unedited.shape[0]

                        f.write(f'{self.objects[object_index].identifier.value} {x} {y} {w} {h}\n')

            print(training_dataset_images)
            print(training_dataset_labels)

    def top_right_click(self):
        print("Top right Clicked")
        self.current_identifier = ObjectsIdentifier((self.current_identifier.value + 1) % NUM_CLASSES)
        if len(self.objects) != 0:
            self.objects[self.current_selection] = self.objects[self.current_selection]._replace(identifier=self.current_identifier)
        self._render_selection()

    def bottom_left_click(self):
        print("Bottom Left Clicked")
        if self.mode == SelectionMode.MANUAL_SEQUENTIAL:
            self.bottom_left.setText("Mode: Manual Closest")
            self.mode = SelectionMode.MANUAL_CLOSEST
        elif self.mode == SelectionMode.MANUAL_CLOSEST:
            self.bottom_left.setText("Mode: AI")
            self.mode = SelectionMode.AI
        elif self.mode == SelectionMode.AI:
            self.bottom_left.setText("Mode: Object Selection")
            self.mode = SelectionMode.OBJECT_SELECTION
        elif self.mode == SelectionMode.OBJECT_SELECTION:
            self.bottom_left.setText("Mode: Manual Sequential")
            self.mode = SelectionMode.MANUAL_SEQUENTIAL

    def bottom_center_click(self):
        print("Selection Has Been Cleared...")
        if len(self.objects) == 0:
            self.current_selection = 0
        else:
            self.objects.pop(self.current_selection)
            self.current_selection = len(self.objects) - 1
        self._render_selection()

    def bottom_right_click(self):
        print("New Object Created...")
        self.objects.append(Object(identifier=self.current_identifier, pts=np.array([[]])))
        self.current_selection = len(self.objects) - 1

    def image_viewer_click(self):
        print(f"Image Viewer Clicked {self.get_cursor_x()} {self.get_cursor_y()}")

        if len(self.objects) == 0:
            self.objects = [Object(identifier=self.current_identifier, pts=np.array([[]]))]

        if self.mode == SelectionMode.AI:
            print("AI Mode")
            results = self.model(self.img_path, points=[[self.get_cursor_x() * self.X_COEFF, self.get_cursor_y() * self.Y_COEFF]], labels=[1])
            
            # Create CV Mat from mask points
            if len(results) != 0 and len(results[0].masks.xy) != 0:
                new_points = np.array(results[0].masks.xy[0], np.int32)
                self.objects[self.current_selection] = self.objects[self.current_selection]._replace(pts=new_points)
                self._render_selection()
        elif self.mode == SelectionMode.MANUAL_SEQUENTIAL:
            print("Manual Sequential Mode")
            new_point = np.array([[self.get_cursor_x() * self.X_COEFF, self.get_cursor_y() * self.Y_COEFF]], np.int32)
            if self.objects[self.current_selection].pts.size == 0:
                self.objects[self.current_selection] = self.objects[self.current_selection]._replace(pts=new_point)
            else:
                self.objects[self.current_selection] = self.objects[self.current_selection]._replace(pts=np.vstack((self.objects[self.current_selection].pts, new_point)))
            self._render_selection()
        elif self.mode == SelectionMode.MANUAL_CLOSEST:
            print("Manual Closest Mode")
            new_point = np.array([[self.get_cursor_x() * self.X_COEFF, self.get_cursor_y() * self.Y_COEFF]], np.int32)
            print(new_point)
            if self.objects[self.current_selection].pts.size == 0:
                self.objects[self.current_selection] = self.objects[self.current_selection]._replace(pts=new_point)
            elif self.objects[self.current_selection].pts.shape[0] == 1:
                self.objects[self.current_selection] = self.objects[self.current_selection]._replace(pts=np.vstack((self.objects[self.current_selection].pts, new_point)))
            else:
                closest_point_distance = np.inf
                closest_point_index = 0

                for i in range(len(self.objects[self.current_selection].pts)):
                    # Line Segment End Points
                    px0, py0 = self.objects[self.current_selection].pts[i]
                    px1, py1 = self.objects[self.current_selection].pts[(i+1) % self.objects[self.current_selection].pts.shape[0]]

                    # Vectors
                    ax, ay = px0 - px1, py0 - py1
                    bx, by = new_point[0][0] - px1, new_point[0][1] - py1

                    # Scaled Projection
                    u = (ax * bx + ay * by) / (ax ** 2 + ay ** 2)
                    u = max(0, min(1, u))

                    cx, cy = px1 + u * ax, py1 + u * ay

                    distance = np.sqrt((new_point[0][0] - cx)**2 + (new_point[0][1] - cy)**2)

                    print(distance)

                    # Check to see if the new point is closer than the other points
                    if distance < closest_point_distance:
                        closest_point_index = i
                        closest_point_distance = distance

                # Determine whether to go one forward or backward
                self.objects[self.current_selection] = self.objects[self.current_selection]._replace(pts=np.insert(self.objects[self.current_selection].pts, (closest_point_index + 1) % self.objects[self.current_selection].pts.shape[0], new_point[0], axis=0))
            self._render_selection()
        elif self.mode == SelectionMode.OBJECT_SELECTION:
            for i, obj in enumerate(self.objects):
                # Perform Horizontal Ray Cast
                cursor_pos = np.array([self.get_cursor_x() * self.X_COEFF, self.get_cursor_y() * self.Y_COEFF], np.int32)
                print(cursor_pos.shape)

                CURSOR_RADIUS = 10

                # If there are no points then we do not want to do any processing
                if obj.pts.shape[1] == 0:
                    continue

                # If there are fewer than three points in the polygon then just look to see if the cursor is close enough to any of the points
                if len(obj.pts) < 3:
                    for (x, y) in obj.pts:
                        distance = np.sqrt((cursor_pos[0] - x) ** 2 + (cursor_pos[1] - y) ** 2)
                        print(distance)

                        if distance < CURSOR_RADIUS:
                            print(f'New selection is {i}')
                            self.current_selection = i
                            self.current_identifier = self.objects[self.current_selection].identifier
                else:       
                    num_intersect = 0
                    for edge_index in range(len(obj.pts)):
                        print(f'Edge Indices {edge_index} {(edge_index + 1) % len(obj.pts)}')
                        if rayintersectseg(cursor_pos, obj.pts[edge_index], obj.pts[(edge_index + 1) % len(obj.pts)]):
                            num_intersect += 1
                            print(f'Intersected {edge_index}')

                    # If there are an odd number of intersections the point is inside the shape
                    if num_intersect % 2 == 1:
                        self.current_selection = i
                        self.current_identifier = self.objects[self.current_selection].identifier
                        print(f'New selection is {i}')
                        break
            self._render_selection()

def main():
    app = QApplication(sys.argv)
    window = ApplicationWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
