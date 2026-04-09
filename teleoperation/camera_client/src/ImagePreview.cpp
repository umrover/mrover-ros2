#include "ImagePreview.hpp"

namespace mrover {

    ImagePreview::ImagePreview(QWidget* parent)
        : QMainWindow(parent),
          mLabel(new QLabel(this)) {

        mLabel->setAlignment(Qt::AlignCenter);
        mLabel->setMinimumSize(640, 480);
        setCentralWidget(mLabel);

        auto* toolbar = addToolBar("Toolbar");

        auto* saveAction = toolbar->addAction("Save");
        connect(saveAction, &QAction::triggered, this, &ImagePreview::saveImage);

        auto* saveAsAction = toolbar->addAction("Save As");
        connect(saveAsAction, &QAction::triggered, this, &ImagePreview::saveImageAs);
    }

    auto ImagePreview::updateImage(QImage const& newImage) -> void {
        mImage = newImage.copy();
        mLabel->setPixmap(QPixmap::fromImage(mImage));
        mLabel->resize(mImage.size());
    }

    void ImagePreview::saveImage() {
        QString const downloads = QStandardPaths::writableLocation(QStandardPaths::DownloadLocation);
        QString const timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        QString const filename = downloads + "/image_" + timestamp + ".png";

        if (!mImage.save(filename)) {
            qWarning("Failed to save image!");
        }
    }

    void ImagePreview::saveImageAs() {
        QString const filename = QFileDialog::getSaveFileName(
                this, "Save Image As",
                QStandardPaths::writableLocation(QStandardPaths::DownloadLocation) + "/image.png",
                "All Files (*);;PNG Images (*.png);;JPEG Images (*.jpg *.jpeg)");

        if (!filename.isEmpty() && !mImage.save(filename)) {
            qWarning("Failed to save image.");
        }
    }

} // namespace mrover
