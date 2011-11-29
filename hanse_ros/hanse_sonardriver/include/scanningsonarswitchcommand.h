#ifndef HANSE_SCANNINGSONARSWITCHCOMMAND_H
#define HANSE_SCANNINGSONARSWITCHCOMMAND_H

#include "sonarswitchcommand.h"

class ScanningSonarSwitchCommand : public SonarSwitchCommand
{
public:
    ScanningSonarSwitchCommand();

    QByteArray toSerialCmd() const;

    unsigned char trainAngle;
    unsigned char sectorWidth;
    unsigned char stepSize;
    unsigned char frequency;
};

#endif
