#include "MoveCameraPopup.hpp"
#include "CameraClientMainWindow.hpp"
#include <format>
#include <qapplication.h>
#include <qboxlayout.h>
#include <qdebug.h>
#include <qdialog.h>
#include <qglobal.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qwidget.h>

#include <qwindowdefs.h>
#include <utility>

#include <utility>

using namespace mrover;

MoveCameraPopup::MoveCameraPopup(std::string const& cameraName, std::string const& pipeline, QWidget* parent) 
: QDialog(parent) {
    layout = new QVBoxLayout(this);
    widget = new QWidget(this);
    std::string const labelString = std::format("Move '{}' to...", cameraName);
    nameLabel = new QLabel(QString::fromStdString(labelString), this);

    layout->addWidget(nameLabel);

    // Get main windows (excluding calling window)
    for(const auto &widget: QApplication::topLevelWidgets()){
        if(auto a = dynamic_cast<CameraClientMainWindow*>(widget)){
            if(widget != parent->window()){
                mainWindows.push_back(dynamic_cast<CameraClientMainWindow*>(widget));
            }
        }
    }

    // Add buttons for windows
    for(const auto &window: mainWindows){
        auto * current_button = new MoveToWindowButton(window->windowTitle(), cameraName, pipeline, this);

        layout->addWidget(current_button);
        connect(current_button, &MoveToWindowButton::pressed, window, [this, cameraName](){
            sendPressed(cameraName);
            accept();
            deleteLater();
        });
        connect(current_button, &MoveToWindowButton::createRequested, window, &CameraClientMainWindow::fullCreateCamera);
    }

    auto * new_window_button = new MoveToWindowButton("New Window (+)", cameraName, pipeline, this, true);
    connect(new_window_button, &MoveToWindowButton::pressed, this, [this, cameraName](){
        createNewWindow("Bingy bongy doodle dabber");
        sendPressed(cameraName);
        accept();
        deleteLater();
    });

    layout->addStretch();
    setLayout(layout);
}

void MoveCameraPopup::sendPressed(std::string name){
    qDebug() << "MoveCameraPopup Sent pressed";
    emit pressed(std::move(name));
}

// TODO check that i don't make a memory leak with this
auto MoveCameraPopup::createNewWindow(QString const& name) -> CameraClientMainWindow*{
    auto new_window = new CameraClientMainWindow();
    new_window->setWindowTitle(name);
    new_window->setMinimumSize(1280, 720);
    new_window->setAttribute(Qt::WA_DeleteOnClose);
    new_window->show();
    qDebug() << "Opened new window '" << name << "'";
    return new_window;
}