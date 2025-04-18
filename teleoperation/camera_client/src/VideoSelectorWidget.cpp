#include "VideoSelectorWidget.hpp"

VideoSelectorWidget::VideoSelectorWidget(QWidget* parent)
    : QWidget(parent) {
    mCheckBoxesLayout = new QVBoxLayout(this);
    mCheckBoxesLayout->setSpacing(10);
    mCheckBoxesLayout->addStretch();
    setLayout(mCheckBoxesLayout);
}

auto VideoSelectorWidget::addSelector(std::string const& name) -> void {
    auto* checkbox = new QCheckBox(QString::fromStdString(name), this);
    checkbox->setChecked(true);

    connect(checkbox, &QCheckBox::toggled, this, [this, name](bool isChecked) {
        emit selectionChanged(name, isChecked);
    });

    // Maintain bottom stretch to keep checkboxes top-aligned
    mCheckBoxesLayout->takeAt(mCheckBoxesLayout->count() - 1);
    mCheckBoxesLayout->addWidget(checkbox);
    mCheckBoxesLayout->addStretch();

    mCheckBoxes.emplace(name, checkbox);
}
