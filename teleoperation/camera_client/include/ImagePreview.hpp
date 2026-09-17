#pragma once

#include "pch.hpp"

namespace mrover {

    class ImagePreview : public QMainWindow {
        Q_OBJECT

        QLabel* mLabel;
        QImage mImage;

    public:
        explicit ImagePreview(QWidget* parent = nullptr);

        auto updateImage(QImage const& newImage) -> void;

    private slots:
        void saveImage();
        void saveImageAs();
    };

} // namespace mrover
