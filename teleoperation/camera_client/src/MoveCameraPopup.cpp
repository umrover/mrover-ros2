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

    long numWindows = QApplication::topLevelWidgets().size();

    for(int i = 0; i < numWindows; ++i){
        layout->addWidget(new QPushButton(QString::fromStdString(std::format("Button {}", i)), this));
    }

    layout->addStretch();
    setLayout(layout);
}