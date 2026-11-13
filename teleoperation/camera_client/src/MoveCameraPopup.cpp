#include "MoveCameraPopup.hpp"
#include "CameraClientMainWindow.hpp"
#include <format>
#include <qapplication.h>
#include <qboxlayout.h>
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

    layout->addStretch();
    setLayout(layout);
}

void MoveCameraPopup::sendPressed(std::string name){
    qDebug() << "MoveCameraPopup Sent pressed";
    emit pressed(std::move(name));
}