import sys

# QT
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton
from PyQt6.QtGui import QIcon, QPixmap, QCursor, QImage
from PyQt6.QtCore import QSize, Qt

# CV
from cv2 import imread

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

def cvmat_to_qpixmap(cvmat):
    height, width, channel = cvmat.shape
    bytesPerLine = 3 * width
    qimg = QImage(cvmat.data, width, height, bytesPerLine, QImage.Format.Format_RGB888).rgbSwapped()
    return QPixmap(qimg)


class ApplicationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setGeometry(STARTING_X_LOCATION, STARTING_Y_LOCATION, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT)
        # TODO: Make the app resizable
        self.setFixedSize(APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT)
        self.init_buttons()
        self.init_image_viewer()

        self.setMouseTracking(True)

    def init_buttons(self):
        # TOP LEFT
        self.top_left = QPushButton("Top Left", self)
        self.top_left.setGeometry(0, 0, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.top_left.clicked.connect(self.top_left_click)

        # TOP CENTER
        self.top_center = QPushButton("Top Center", self)
        self.top_center.setGeometry(BUTTON_WIDTH, 0, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.top_center.clicked.connect(self.top_center_click)

        # TOP RIGHT
        self.top_right = QPushButton("Top Right", self)
        self.top_right.setGeometry(2 * BUTTON_WIDTH, 0, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.top_right.clicked.connect(self.top_right_click)

        # BOTTOM LEFT
        self.bottom_left = QPushButton("Bottom Left", self)
        self.bottom_left.setGeometry(0, self.height() - BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.bottom_left.clicked.connect(self.bottom_left_click)

        # TOP CENTER
        self.bottom_center = QPushButton("Bottom Center", self)
        self.bottom_center.setGeometry(BUTTON_WIDTH, self.height() - BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.bottom_center.clicked.connect(self.bottom_center_click)

        # TOP RIGHT
        self.bottom_right = QPushButton("Bottom Right", self)
        self.bottom_right.setGeometry(2 * BUTTON_WIDTH, self.height() - BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT)
        self.bottom_right.clicked.connect(self.bottom_right_click)

    def init_image_viewer(self):
        self.image_viewer_button = QPushButton(self)
        self.image_viewer_button.setGeometry(0, BUTTON_HEIGHT, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT - 2 * BUTTON_HEIGHT)
        self.image_viewer_button.clicked.connect(self.image_viewer_click)

        self.image_viewer_label = QLabel(self)
        self.image_viewer_label.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents) # allows the click to hit the button instead of the label
        self.image_viewer_label.setGeometry(0, BUTTON_HEIGHT, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT - 2 * BUTTON_HEIGHT)

        cvmat = imread(IMAGE_PATH)

        self.image_viewer_pixmap = cvmat_to_qpixmap(cvmat)
        self.image_viewer_label.setPixmap(self.image_viewer_pixmap)
        self.image_viewer_label.setScaledContents(True)

    def top_left_click(self):
        print("Top Left Clicked")

    def top_center_click(self):
        print("Top center Clicked")

    def top_right_click(self):
        print("Top right Clicked")

    def bottom_left_click(self):
        print("Bottom Left Clicked")

    def bottom_center_click(self):
        print("Bottom center Clicked")

    def bottom_right_click(self):
        print("Bottom right Clicked")

    def get_cursor_x(self):
        return self.mapFromGlobal(QCursor.pos()).x()

    def get_cursor_y(self):
        return self.mapFromGlobal(QCursor.pos()).y() - BUTTON_HEIGHT

    def image_viewer_click(self):
        print(f"Image Viewer Clicked {self.get_cursor_x()} {self.get_cursor_y()}")

def main():
    app = QApplication(sys.argv)
    window = ApplicationWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
