#include "sonarswitchcommand.h"

SonarSwitchCommand::SonarSwitchCommand()
{
    time = QDateTime::currentDateTime();

    // all fields remain uninitialized. must be done from the outside
    // or via = operator
}

SonarSwitchCommand::SonarSwitchCommand(QByteArray fileData)
{
    extractHeader(fileData);
}

SonarSwitchCommand::SonarSwitchCommand(const SonarSwitchCommand& other)
{
    clone(other);
}

QByteArray SonarSwitchCommand::toSerialCmd()
{
    QByteArray a;
    a.resize(27);
    a[0] = 0xFE;        // Switch data header
    a[1] = 0x44;        // Switch data header
    a[2] = 0x10;        // Head ID
    a[3] = range;       // Range
    a[4] = 0;           // Reserved
    a[5] = 0;           // Reserved
    a[6] = 0;           // Reserved
    a[7] = 0;           // Reserved
    a[8] = startGain;   // Start Gain
    a[9] = 0;           // Reserved
    a[10]= 20;          // Absorption
    a[11]= trainAngle;  // Train Angle
    a[12]= sectorWidth; // Sector Width
    a[13]= stepSize;    // Step Size
    a[14]= pulseLength; // PulseLength
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
    a[25] = frequency;  // Frequency
    a[26] = 0xFD;       // Termination Byte

    return a;
}

SonarSwitchCommand& SonarSwitchCommand::operator =(SonarSwitchCommand other) {
    clone(other);
    return *this;
}

void SonarSwitchCommand::extractHeader(const QByteArray &a)
{
/*    if (a.length()<100)
        logger->error("852 header not long enough: "+QString::number(a.length()));
*/
    QDataStream stream(a);

    stream.skipRawData(4);

    stream >> totalBytes >> nToRead;
    char date[12];
    stream.readRawData(date, 12);
    char time[9];
    stream.readRawData(time, 9);
    char hs[4];
    stream.readRawData(hs, 4);

    QString fullString(date);
    fullString.append(" ");
    fullString.append(time);
    fullString.append(hs);

    this->time = QLocale(QLocale::English, QLocale::UnitedKingdom).toDateTime(fullString, "dd-MMM-yyyy HH:mm:ss.z");

    stream.skipRawData(4);

    unsigned char flags;
    stream >> flags;

    this->reversedDirection = flags & 0x80;
    this->stepSize = flags & 0x07;

    stream >> startGain;
    stream >> sectorWidth;
    stream >> trainAngle;

    stream.skipRawData(3);

    stream >> pulseLength;

    stream.skipRawData(42);

    stream >> frequency;

    unsigned char headID;

    stream >> headID;

    if (headID != 0x10) {
        //   logger->error("read bad value as head ID. parser error?");
    }
//    logger->trace("totalBytes=" + QString::number(totalBytes));
//    logger->trace("nToRead=" + QString::number(nToRead));
//    logger->trace("startGain=" + QString::number(startGain));
//    logger->trace("DateTime=" + this->time.toString("ddd MMM d yyyy HH:mm:ss.zzz"));

    origFileHeader = a;
}

void SonarSwitchCommand::clone(const SonarSwitchCommand &other)
{
    this->time = other.time;
    this->range = other.range;
    this->reversedDirection = other.reversedDirection;
    this->startGain = other.startGain;
    this->trainAngle = other.trainAngle;
    this->sectorWidth = other.sectorWidth;
    this->stepSize = other.stepSize;
    this->pulseLength = other.pulseLength;
    this->dataPoints = other.dataPoints;
    this->switchDelay = other.switchDelay;
    this->frequency = other.frequency;
    this->totalBytes = other.totalBytes;
    this->nToRead = other.nToRead;
    this->origFileHeader = other.origFileHeader;
}
