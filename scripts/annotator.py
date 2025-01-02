import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QPushButton
from PyQt5.QtGui import QPixmap

# WINDOW CONSTANTS
STARTING_X_LOCATION = 200
STARTING_Y_LOCATION = 100

APP_WINDOW_WIDTH = 1280
APP_WINDOW_HEIGHT = 720

# LAYOUT CONSTANTS
BUTTON_HEIGHT = 150
BUTTON_WIDTH = 426

class ApplicationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setGeometry(STARTING_X_LOCATION, STARTING_Y_LOCATION, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT)
        # TODO: Make the app resizable
        self.setFixedSize(APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT)
        self.init_buttons()
        self.init_image_viewer()

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
        self.image_viewer_label = QLabel(self)
        self.image_viewer_label.setGeometry(0, BUTTON_HEIGHT, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT - 2 * BUTTON_HEIGHT)

        self.image_viewer_pixmap = QPixmap("data/images/pic.png")
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

def main():
    app = QApplication(sys.argv)
    window = ApplicationWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
