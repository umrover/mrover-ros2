#pragma once

#include <QPushButton>

namespace mrover {

    using RequestCallback = std::function<bool()>;

    class CallbackCheckBox : public QPushButton {
        Q_OBJECT

        QIcon mUncheckedIcon;
        QIcon mCheckedIcon;
        QString mUncheckedText;
        QString mCheckedText;

        bool mUsingIcons;
        bool mChecked{false};

        RequestCallback mOnCheckCallback;
        RequestCallback mOnUncheckCallback;

        auto updateAppearance() -> void;
        auto handleClick() -> void;

    public:
        explicit CallbackCheckBox(QString uncheckedText, QString checkedText, QWidget* parent = nullptr);
        explicit CallbackCheckBox(QIcon uncheckedIcon, QIcon checkedIcon, QWidget* parent = nullptr);

        auto setChecked(bool checked) -> void;
        auto setOnCheckCallback(RequestCallback callback) -> void;
        auto setOnUncheckCallback(RequestCallback callback) -> void;
    };

} // namespace mrover
