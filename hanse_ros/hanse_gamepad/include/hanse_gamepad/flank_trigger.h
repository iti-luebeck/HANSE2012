#include <vector>
#include <ros/types.h>

#ifndef FLANK_TRIGGER_H
#define FLANK_TRIGGER_H

class FlankTrigger
{
public:
    FlankTrigger();

    void setActual(std::vector<int32_t> actualButtons);
    bool isSet(uint8_t button);

private:
    std::vector<int32_t> lastButtons;
    std::vector<int32_t> actualButtons;
};

#endif // FLANK_TRIGGER_H
