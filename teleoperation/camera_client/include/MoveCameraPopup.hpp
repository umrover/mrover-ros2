#pragma once

#include "CameraClientMainWindow.hpp"
#include "pch.hpp"
#include <qboxlayout.h>
#include <qdialog.h>
#include <qglobal.h>
#include <qpushbutton.h>
#include <qtmetamacros.h>
#include <qwidget.h>

namespace mrover {
    class MoveToWindowButton : public QPushButton {
        Q_OBJECT

        public:
            explicit MoveToWindowButton(
                const QString& title, 
                const std::string& cameraName, 
                const std::string& pipeline, 
                QWidget* parent = nullptr,
                bool open_new_window = false
            ): QPushButton(title, parent), open_new(open_new_window) {
                connect(this, &QPushButton::pressed, this, [this, cameraName, pipeline]() {
                    MoveToWindowButton::createRequested(cameraName, pipeline);
                });
            }

            void sendCreateRequest(std::string name, std::string pipeline){
                qDebug() << "Sending create request";
                emit createRequested(std::move(name), std::move(pipeline));
            }
        
        private:
            bool open_new;

        signals:
            void createRequested(std::string name, std::string pipeline);
    };
    class MoveCameraPopup : public QDialog {
        Q_OBJECT

        QWidget* widget;
        QVBoxLayout* layout;
        
        QLabel* nameLabel;
        
    
    public:
        explicit MoveCameraPopup(std::string const& cameraName, std::string const& pipeline, QWidget* parent = nullptr);
        static auto createNewWindow(QString const& name) -> CameraClientMainWindow*;

    signals:
        void createRequested(std::string name, std::string pipeline);
        void pressed(std::string name);
    
    private:
        std::vector<CameraClientMainWindow*> mainWindows;
    
    private slots:

        // void sendCreateRequest(std::string name, std::string pipeline);
        void sendPressed(std::string name);
        
    };    
} // namespace mrover