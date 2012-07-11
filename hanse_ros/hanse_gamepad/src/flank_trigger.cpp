#include "hanse_gamepad/flank_trigger.h"

void FlankTrigger::setActual(std::vector<int32_t> buttons)
{
    last_buttons_ = actual_buttons_;
    actual_buttons_ = buttons;


    if (last_buttons_.size() != actual_buttons_.size())
    {
        last_buttons_.resize(actual_buttons_.size(), 0);
    }
}

bool FlankTrigger::isSet(uint8_t button)
{
    return (last_buttons_[button] == 0 && actual_buttons_[button] == 1);
}
