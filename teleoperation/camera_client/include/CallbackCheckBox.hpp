#pragma once

#include <QPushButton>

namespace mrover {
    using RequestCallback = std::function<bool()>;
    constexpr auto DEFAULT_REQUEST_CALLBACK = []() { return true; };

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

    public:
        explicit CallbackCheckBox(QString const& uncheckedText, QString const& checkedText, QWidget* parent = nullptr)
            : QPushButton(parent),
              mUncheckedText(uncheckedText),
              mCheckedText(checkedText),
              mUsingIcons(false) {

            setText(uncheckedText);

            mOnCheckCallback = DEFAULT_REQUEST_CALLBACK;
            mOnUncheckCallback = DEFAULT_REQUEST_CALLBACK;

            connect(this, &QPushButton::clicked, this, [this]() {
                if (mChecked) {
                    if (!mOnUncheckCallback()) {
                        return;
                    }
                } else {
                    if (!mOnCheckCallback()) {
                        return;
                    }
                }
                mChecked = !mChecked;
                setText(mChecked ? mCheckedText : mUncheckedText);
            });
        }
        explicit CallbackCheckBox(QIcon const& uncheckedIcon, QIcon const& checkedIcon, QWidget* parent = nullptr)
            : QPushButton(parent),
              mUncheckedIcon(uncheckedIcon),
              mCheckedIcon(checkedIcon),
              mUsingIcons(true) {

            setIcon(uncheckedIcon);

            mOnCheckCallback = DEFAULT_REQUEST_CALLBACK;
            mOnUncheckCallback = DEFAULT_REQUEST_CALLBACK;

            connect(this, &QPushButton::clicked, this, [this]() {
                if (mChecked) {
                    if (!mOnUncheckCallback()) {
                        return;
                    }
                } else {
                    if (!mOnCheckCallback()) {
                        return;
                    }
                }
                mChecked = !mChecked;
                setIcon(mChecked ? mCheckedIcon : mUncheckedIcon);
            });
        }

        void setChecked(bool checked) {
            mChecked = checked;
            if (mUsingIcons) {
                setIcon(mChecked ? mCheckedIcon : mUncheckedIcon);
            } else {
                setText(mChecked ? mCheckedText : mUncheckedText);
            }
        }

        void setOnCheckCallback(RequestCallback callback) {
            mOnCheckCallback = std::move(callback);
            connect(this, &QPushButton::clicked, this, [this]() {
                if (mChecked) {
                    if (!mOnUncheckCallback()) {
                        return;
                    }
                } else {
                    if (!mOnCheckCallback()) {
                        return;
                    }
                }
                mChecked = !mChecked;
                setIcon(mChecked ? mCheckedIcon : mUncheckedIcon);
            });
        }
        void setOnUncheckCallback(RequestCallback callback) {
            mOnUncheckCallback = std::move(callback);
            connect(this, &QPushButton::clicked, this, [this]() {
                if (mChecked) {
                    if (!mOnUncheckCallback()) {
                        return;
                    }
                } else {
                    if (!mOnCheckCallback()) {
                        return;
                    }
                }
                mChecked = !mChecked;
                setIcon(mChecked ? mCheckedIcon : mUncheckedIcon);
            });
        }
    };

} // namespace mrover
