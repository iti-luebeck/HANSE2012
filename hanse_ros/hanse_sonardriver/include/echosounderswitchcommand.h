#ifndef HANSE_ECHOSOUNDERSWITCHCOMMAND_H
#define HANSE_ECHOSOUNDERSWITCHCOMMAND_H

#include "sonarswitchcommand.h"

class EchoSounderSwitchCommand : public SonarSwitchCommand
{
public:
    EchoSounderSwitchCommand();

    QByteArray toSerialCmd() const;

    unsigned char profileMinimumRange;
    bool profile;

};

#endif
