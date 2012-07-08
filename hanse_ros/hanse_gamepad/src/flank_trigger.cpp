
#include "hanse_gamepad/flank_trigger.h"

FlankTrigger::FlankTrigger() {}

void FlankTrigger::setActual(std::vector<int32_t> actualButtons)
{
    this->lastButtons = this->actualButtons;
    this->actualButtons = actualButtons;

    lastButtons.resize(15);
    actualButtons.resize(15);
}

bool FlankTrigger::isSet(uint8_t button)
{
    return (lastButtons[button] == 0 && actualButtons[button] == 1);
}
