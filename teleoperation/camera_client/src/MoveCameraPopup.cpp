#include "MoveCameraPopup.hpp"
#include "CameraClientMainWindow.hpp"
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
    nameLabel = new QLabel("test text", this);

    layout->addWidget(nameLabel);

    // Get number of main windows
    // long numWindows = 0;
    for(const auto &widget: QApplication::topLevelWidgets()){
        if(auto a = dynamic_cast<CameraClientMainWindow*>(widget)/*typeid(widget) == typeid(CameraClientMainWindow)*/){
            mainWindows.push_back(dynamic_cast<CameraClientMainWindow*>(widget));
            // ++numWindows;
        }
    }

    CameraClientMainWindow * firstCam = mainWindows.front();

    // Add buttons for windows
    for(const auto &window: mainWindows /*size_t i = 0; i < mainWindows.size(); ++i*/){
        auto * current_button = new MoveToWindowButton(window->windowTitle(), cameraName, pipeline, this);

        // connect(current_button, &QPushButton::pressed, this, [this](/*std::string const& name, std::string const& pipeline*/) {
        //     sendCreateRequest("test name", "test pipeline");
        // });

        layout->addWidget(current_button);
        connect(current_button, &MoveToWindowButton::pressed, window, [this, cameraName](){
            sendPressed(cameraName);
            accept();
            // deleteLater();
        });
        connect(current_button, &MoveToWindowButton::createRequested, window, &CameraClientMainWindow::fullCreateCamera);

        // mrover::CameraClientMainWindow::connect(this, &MoveCameraPopup::createRequested, firstCam, &CameraClientMainWindow::fullCreateCamera);
    }

    layout->addStretch();
    setLayout(layout);
}

// void MoveCameraPopup::sendCreateRequest(std::string name, std::string pipeline){
//     emit createRequested(std::move(name), std::move(pipeline));
// }

void MoveCameraPopup::sendPressed(std::string name){
    qDebug() << "MoveCameraPopup Sent pressed";
    emit pressed(std::move(name));
}