#pragma once

#include "pch.hpp"

class VideoSelectorWidget : public QWidget {
    Q_OBJECT

    std::unordered_map<std::string, QCheckBox*> mCheckBoxes;
    QVBoxLayout* mCheckBoxesLayout;

public:
    explicit VideoSelectorWidget(QWidget* parent = nullptr);

    auto addSelector(std::string const& name, QWidget* targetWidget) -> void;
};
