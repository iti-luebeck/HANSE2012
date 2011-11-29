#include "scanningsonarswitchcommand.h"

ScanningSonarSwitchCommand::ScanningSonarSwitchCommand()
{
    headId = 0x10;
}

QByteArray ScanningSonarSwitchCommand::toSerialCmd() const
{
    QByteArray a = SonarSwitchCommand::toSerialCmd();
    a[11] = trainAngle;  // Train Angle
    a[12] = sectorWidth; // Sector Width
    a[13] = stepSize;    // Step Size
    a[25] = frequency;  // Frequency
    return a;
}
