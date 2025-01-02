import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel

STARTING_X_LOCATION = 200
STARTING_Y_LOCATION = 100

APP_WINDOW_WIDTH = 1280
APP_WINDOW_HEIGHT = 720

class ApplicationWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setGeometry(STARTING_X_LOCATION, STARTING_Y_LOCATION, APP_WINDOW_WIDTH, APP_WINDOW_HEIGHT)

def main():
    app = QApplication(sys.argv)
    window = ApplicationWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
