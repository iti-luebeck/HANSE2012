#ifndef SONARSWITCHCOMMAND_H
#define SONARSWITCHCOMMAND_H

#include <QtCore>

class SonarSwitchCommand
{
public:
    // when building it by hand (serial)
    SonarSwitchCommand();

    SonarSwitchCommand(const SonarSwitchCommand&);

    virtual QByteArray toSerialCmd() const;
    SonarSwitchCommand& operator =(const SonarSwitchCommand&);

    QDateTime time;
    unsigned char range;
    unsigned char startGain;
    unsigned char pulseLength;
    unsigned char dataPoints;
    unsigned char switchDelay;
    unsigned short totalBytes;
    unsigned short nToRead;

private:
    void clone(const SonarSwitchCommand& other);
};

#endif // SONARSWITCHCOMMAND_H
