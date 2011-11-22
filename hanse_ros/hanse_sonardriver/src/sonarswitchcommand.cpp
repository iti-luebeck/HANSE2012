#include "sonarswitchcommand.h"

SonarSwitchCommand::SonarSwitchCommand()
{
    time = QDateTime::currentDateTime();

    // all fields remain uninitialized. must be done from the outside
    // or via = operator
}

SonarSwitchCommand::SonarSwitchCommand(const SonarSwitchCommand& other)
{
    clone(other);
}


QByteArray SonarSwitchCommand::toSerialCmd() const
{
    QByteArray a;
    a.resize(27);
    a[0]  = 0xFE;        // Switch data header
    a[1]  = 0x44;        // Switch data header
    a[2]  = 0;        // Head ID
    a[3]  = range;       // Range
    a[4]  = 0;           // Reserved
    a[5]  = 0;           // Reverse direction
    a[6]  = 0;           // Reserved
    a[7]  = 0;           // Reserved
    a[8]  = startGain;   // Start Gain
    a[9]  = 0;           // Reserved
    a[10] = 20;          // Absorption
    a[11] = 0;           // Reserved
    a[12] = 0;           // Reserved
    a[13] = 0;           // Reserved
    a[14] = pulseLength; // PulseLength
    a[15] = 0;          // Reserved
    a[16] = 0;          // Reserved
    a[17] = 0;          // Reserved
    a[18] = 0;          // Reserved
    a[19] = dataPoints; // DataPoints: 25 or 50
    a[20] = 0;          // Reserved
    a[21] = 0;          // Reserved
    a[22] = 0;          // Reserved
    a[23] = 0;          // Reserved
    a[24] = switchDelay;// Switch delay
    a[25] = 0;          // Reserved
    a[26] = 0xFD;       // Termination Byte

    return a;
}


SonarSwitchCommand& SonarSwitchCommand::operator =(const SonarSwitchCommand &other) {
    clone(other);
    return *this;
}

void SonarSwitchCommand::clone(const SonarSwitchCommand &other)
{
    this->time = other.time;
    this->range = other.range;
    this->startGain = other.startGain;
    this->pulseLength = other.pulseLength;
    this->dataPoints = other.dataPoints;
    this->switchDelay = other.switchDelay;
    this->totalBytes = other.totalBytes;
    this->nToRead = other.nToRead;
}
