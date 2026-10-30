#include "MoveCameraPopup.hpp"
#include <qboxlayout.h>
#include <qdialog.h>
#include <qlabel.h>
#include <qwidget.h>

using namespace mrover;

MoveCameraPopup::MoveCameraPopup(QWidget* parent) 
: QDialog(parent) {
    layout = new QVBoxLayout(this);
    widget = new QWidget(this);
    nameLabel = new QLabel("test text", this);

    layout->addWidget(nameLabel);
    layout->addStretch();
    setLayout(layout);
}