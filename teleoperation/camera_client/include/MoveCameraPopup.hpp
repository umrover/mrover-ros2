#pragma once

#include "CameraClientMainWindow.hpp"
#include "pch.hpp"
#include <qboxlayout.h>
#include <qdialog.h>
#include <qpushbutton.h>
#include <qtmetamacros.h>
#include <qwidget.h>

namespace mrover {
    class MoveCameraPopup : public QDialog {
        Q_OBJECT

        QWidget* widget;
        QVBoxLayout* layout;
        
        QLabel* nameLabel;
        
    
    public:
        explicit MoveCameraPopup(QWidget* parent = nullptr);

    signals:
        void createRequested(std::string name, std::string pipeline);
    
    private:
        std::vector<CameraClientMainWindow*> mainWindows;
    
    private slots:

        void sendCreateRequest(std::string name, std::string pipeline);
        
    };    
} // namespace mrover