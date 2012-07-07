
#include "hanse_gamepad/flank_trigger.h"

FlankTrigger::FlankTrigger() {}

void FlankTrigger::setActual(std::vector<int32_t> actualButtons)
{
    this->lastButtons = this->actualButtons;
    this->actualButtons = actualButtons;

    if (lastButtons.size() != actualButtons.size()) {
        lastButtons.resize(actualButtons.size());
    }
}

bool FlankTrigger::isSet(uint8_t button)
{
    return !lastButtons[button] && actualButtons[button];
}
