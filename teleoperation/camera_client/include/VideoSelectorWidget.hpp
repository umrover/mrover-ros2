#pragma once

#include <QCheckBox>
#include <QDockWidget>
#include <QVBoxLayout>


class VideoSelectorWidget : public QWidget {
    Q_OBJECT

    std::unordered_map<std::string, QCheckBox*> mCheckBoxes;
    QVBoxLayout* mCheckBoxesLayout;

public:
    explicit VideoSelectorWidget(QWidget* parent = nullptr) : QWidget(parent) {
        mCheckBoxesLayout = new QVBoxLayout(this);
        mCheckBoxesLayout->setSpacing(10);
        mCheckBoxesLayout->addStretch();

        setLayout(mCheckBoxesLayout);
    }

    auto addSelector(std::string const& name, QWidget* targetWidget) -> void {
        auto* checkbox = new QCheckBox(QString::fromStdString(name), this);
        checkbox->setChecked(true);
        connect(checkbox, &QCheckBox::toggled, targetWidget, [targetWidget](bool isUnchecked) {
            targetWidget->setVisible(isUnchecked);
        });
        // remove the spacer at the end of the layout, add the checkbox, and add the spacer again (to keep the checkboxes at the top)
        mCheckBoxesLayout->takeAt(mCheckBoxesLayout->count() - 1);
        mCheckBoxesLayout->addWidget(checkbox);
        mCheckBoxesLayout->addStretch();

        mCheckBoxes.emplace(name, checkbox);
    }
};
