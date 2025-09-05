#pragma once

#include "pch.hpp"

class ImagePreview : public QMainWindow {
    Q_OBJECT

    QLabel* label;
    QImage image;
    QString name;

public:
    explicit ImagePreview(QWidget* parent = nullptr) : QMainWindow(parent), label(new QLabel(this)) {

        label->setAlignment(Qt::AlignCenter);
        label->setMinimumSize(640, 480);
        setCentralWidget(label);

        auto* toolbar = addToolBar("Toolbar");

        auto* saveAction = toolbar->addAction("Save");
        connect(saveAction, &QAction::triggered, this, &ImagePreview::saveImage);

        auto* saveAsAction = toolbar->addAction("Save As");
        connect(saveAsAction, &QAction::triggered, this, &ImagePreview::saveImageAs);
    }

    auto updateImage(QImage const& newImage) -> void {
        image = newImage.copy();
        label->setPixmap(QPixmap::fromImage(image));
        label->resize(image.size());
    }

private slots:
    void saveImage() {
        QString const downloads = QStandardPaths::writableLocation(QStandardPaths::DownloadLocation);
        QString const timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
        QString const filename = downloads + "/image_" + timestamp + ".png";

        if (!image.save(filename)) {
            qWarning("Failed to save image!");
        }
    }

    void saveImageAs() {
        QString const filename = QFileDialog::getSaveFileName(
                this, "Save Image As", QStandardPaths::writableLocation(QStandardPaths::DownloadLocation) + "/image.png",
                "All Files (*);;PNG Images (*.png);;JPEG Images (*.jpg *.jpeg)");

        if (!filename.isEmpty() && !image.save(filename)) {
            qWarning("Failed to save image.");
        }
    }
};
