#ifndef SONARSWITCHCOMMAND_H
#define SONARSWITCHCOMMAND_H

#include <QtCore>

class SonarSwitchCommand
{
public:
    // when reading from file
    SonarSwitchCommand(QByteArray fileData);

    // when building it by hand (serial)
    SonarSwitchCommand();

    SonarSwitchCommand(const SonarSwitchCommand&);

    QByteArray toSerialCmd();
    SonarSwitchCommand& operator =(SonarSwitchCommand);

    QDateTime time;
    unsigned char range;
    bool reversedDirection;
    unsigned char startGain;
    unsigned char trainAngle;
    unsigned char sectorWidth;
    unsigned char stepSize;
    unsigned char pulseLength;
    unsigned char dataPoints;
    unsigned char switchDelay;
    unsigned char frequency;
    unsigned short totalBytes;
    unsigned short nToRead;

    QByteArray origFileHeader;


private:
    void extractHeader(const QByteArray& a);
    void clone(const SonarSwitchCommand& other);
};

#endif // SONARSWITCHCOMMAND_H
