#include <QApplication>

#include "MainWindow.hpp"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    MainWindow mainWindow;
    mainWindow.setWindowTitle("MRover Cameras");
    mainWindow.setMinimumSize(1280, 720);
    mainWindow.show();

    return app.exec();
}
