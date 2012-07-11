#ifndef FLANK_TRIGGER_H
#define FLANK_TRIGGER_H

#include <vector>
#include <ros/types.h>
#include <iostream>

/**
 * \brief Class for flank triggering of gamepad buttons.
 *
 * All gamepad buttons are validated via rising flank
 * detection. With this flank triggering multiple
 * button signals are avoided when multiple buttons
 * are pressed at the same time.
 */
class FlankTrigger
{
public:
    /**
     * \brief Sets the actual sended buttons.
     *
     * Save the actual pressed and released buttons.
     *
     * \param actual_buttons vector of buttons
     */
    void setActual(std::vector<int32_t> actual_buttons);
    bool isSet(uint8_t button);

private:
    std::vector<int32_t> last_buttons_;
    std::vector<int32_t> actual_buttons_;
};

#endif // FLANK_TRIGGER_H
