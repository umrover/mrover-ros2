#include "MoveCameraPopup.hpp"
#include <qapplication.h>
#include <qboxlayout.h>
#include <qdialog.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qwidget.h>

using namespace mrover;

MoveCameraPopup::MoveCameraPopup(QWidget* parent) 
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

    // Add buttons for windows
    for(const auto &window: mainWindows /*size_t i = 0; i < mainWindows.size(); ++i*/){
        layout->addWidget(new QPushButton(window->windowTitle(), this));
    }

    layout->addStretch();
    setLayout(layout);
}