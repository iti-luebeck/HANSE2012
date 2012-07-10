#ifndef FLANK_TRIGGER_H
#define FLANK_TRIGGER_H

#include <vector>
#include <ros/types.h>
#include <iostream>

class FlankTrigger
{
public:
    void setActual(std::vector<int32_t> actualButtons);
    bool isSet(uint8_t button);

private:
    std::vector<int32_t> lastButtons;
    std::vector<int32_t> actualButtons;
};

#endif // FLANK_TRIGGER_H
