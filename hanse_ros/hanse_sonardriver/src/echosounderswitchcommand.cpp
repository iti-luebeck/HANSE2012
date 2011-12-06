#include "echosounderswitchcommand.h"

EchoSounderSwitchCommand::EchoSounderSwitchCommand()
{
    headId = 0x11;
}

QByteArray EchoSounderSwitchCommand::toSerialCmd() const
{
    QByteArray a = SonarSwitchCommand::toSerialCmd();
    a[10] = 20; // Absorption
    a[15] = profileMinimumRange; // Profile minimum range
    a[22] = profile ? 1 : 0; // Profile
    return a;
}
