#include "CallbackCheckBox.hpp"

namespace mrover {

    namespace {
        constexpr auto DEFAULT_REQUEST_CALLBACK = []() { return true; };
    } // namespace

    CallbackCheckBox::CallbackCheckBox(QString uncheckedText, QString checkedText, QWidget* parent)
        : QPushButton(parent),
          mUncheckedText(std::move(uncheckedText)),
          mCheckedText(std::move(checkedText)),
          mUsingIcons(false),
          mOnCheckCallback(DEFAULT_REQUEST_CALLBACK),
          mOnUncheckCallback(DEFAULT_REQUEST_CALLBACK) {

        setText(mUncheckedText);
        connect(this, &QPushButton::clicked, this, &CallbackCheckBox::handleClick);
    }

    CallbackCheckBox::CallbackCheckBox(QIcon uncheckedIcon, QIcon checkedIcon, QWidget* parent)
        : QPushButton(parent),
          mUncheckedIcon(std::move(uncheckedIcon)),
          mCheckedIcon(std::move(checkedIcon)),
          mUsingIcons(true),
          mOnCheckCallback(DEFAULT_REQUEST_CALLBACK),
          mOnUncheckCallback(DEFAULT_REQUEST_CALLBACK) {

        setIcon(mUncheckedIcon);
        connect(this, &QPushButton::clicked, this, &CallbackCheckBox::handleClick);
    }

    auto CallbackCheckBox::handleClick() -> void {
        setDisabled(true);

        bool const shouldToggle = mChecked ? mOnUncheckCallback() : mOnCheckCallback();
        if (shouldToggle) {
            mChecked = !mChecked;
            updateAppearance();
        }

        setDisabled(false);
    }

    auto CallbackCheckBox::updateAppearance() -> void {
        if (mUsingIcons) {
            setIcon(mChecked ? mCheckedIcon : mUncheckedIcon);
        } else {
            setText(mChecked ? mCheckedText : mUncheckedText);
        }
    }

    auto CallbackCheckBox::setChecked(bool checked) -> void {
        mChecked = checked;
        updateAppearance();
    }

    auto CallbackCheckBox::setOnCheckCallback(RequestCallback callback) -> void {
        mOnCheckCallback = std::move(callback);
    }

    auto CallbackCheckBox::setOnUncheckCallback(RequestCallback callback) -> void {
        mOnUncheckCallback = std::move(callback);
    }

} // namespace mrover
