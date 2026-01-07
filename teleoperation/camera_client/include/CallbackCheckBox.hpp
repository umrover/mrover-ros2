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
                this->setDisabled(true);
                if (mChecked) {
                    if (!mOnUncheckCallback()) {
                        goto end;
                    }
                } else {
                    if (!mOnCheckCallback()) {
                        goto end;
                    }
                }
                mChecked = !mChecked;
                setText(mChecked ? mCheckedText : mUncheckedText);
            end:
                this->setDisabled(false);
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
                this->setDisabled(true);
                if (mChecked) {
                    if (!mOnUncheckCallback()) {
                        goto end;
                    }
                } else {
                    if (!mOnCheckCallback()) {
                        goto end;
                    }
                }
                mChecked = !mChecked;
                setIcon(mChecked ? mCheckedIcon : mUncheckedIcon);
            end:
                this->setDisabled(false);
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
        }
        void setOnUncheckCallback(RequestCallback callback) {
            mOnUncheckCallback = std::move(callback);
        }
    };

} // namespace mrover
