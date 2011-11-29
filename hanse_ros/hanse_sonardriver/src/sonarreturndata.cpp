#include "sonarreturndata.h"
#include "QtCore"

SonarReturnData::SonarReturnData()
{
}

SonarReturnData::SonarReturnData(const SonarReturnData& c)
{
    this->packet = c.packet;
    this->switchCommand = c.switchCommand;
    this->headPos = c.headPos;
    this->valid = c.valid;
    this->echo = c.echo;
    this->range = c.range;
}

SonarReturnData::SonarReturnData(const SonarSwitchCommand& cmd, QByteArray& returnDataPacket)
{
    this->packet = returnDataPacket;
    this->switchCommand = cmd;

    valid = true;
    if (packet.size()<8) {
        valid = false;
    } else if (packet[0] != 'I' || packet[2] != 'X') {
        // ID Bytes
        valid = false;
    } else if (packet.size() < 8) {
        valid = false;
    } else if ((char)packet[3] != cmd.getHeadId()) {
        valid = false;
    } else if (!isSwitchesAccepted()) {
        valid = false;
    } else if (isCharacterOverrun()) {
        valid = false;
    } else if (packet.length() == 265) {
        if (packet[1] != 'M') {
            valid = false;
        } else if (getDataBytes() != 252) {
            valid = false;
        }
    } else if (packet.length() == 513) {
        if (packet[1] != 'G') {
            valid = false;
        } else if (getDataBytes() != 500) {
            valid = false;
        }
    } else if (packet.length() == 13) {
        if (packet[1] != 'P') {
            valid = false;
        } else if (getDataBytes() != 0) {
            valid = false;
        }
    }

    // Termination byte
    if (valid && packet[packet.length()-1] != (char)0xFC)
        valid = false;


    if (valid) {
        echo = packet;
        echo.remove(0,12); // remove header
        echo.remove(getDataBytes(), 1); // remove termination byte

        headPos = THCHeadPosDecoder(packet[5], packet[6]);
        headPos = (0.15*(headPos - 1400));
        headPos += 90;
        //+90 offset

        range = packet[7];

        profileRange = THCDecoder(packet[8], packet[9]);
    }
}

SonarReturnData& SonarReturnData::operator =(SonarReturnData other) {
    this->packet = other.packet;
    this->switchCommand = other.switchCommand;
    this->headPos = other.headPos;
    this->valid = other.valid;
    this->echo = other.echo;
    this->range = other.range;

    return *this;
}

int SonarReturnData::THCDecoder(char byteLO, char byteHI) const
{
    int high =   (byteHI & 0x7E) >> 1;
    int low = ( ((byteHI & 0x01) << 7) | (byteLO & 0x7F) );

    return high << 8 | low;
}

int SonarReturnData::THCHeadPosDecoder(char byteLO, char byteHI) const
{
    int high =   (byteHI & 0x3E) >> 1;
    int low = ( ((byteHI & 0x01) << 7) | (byteLO & 0x7F) );

    return high << 8 | low;
}

bool SonarReturnData::isSwitchesAccepted() const
{
    return (packet[4] & 0x40) >> 6;
}

bool SonarReturnData::isCharacterOverrun() const
{
    return (packet[4] & 0x80) >> 7;
}

bool SonarReturnData::isCWDirection() const
{
    return (packet[6] & 0x40) >> 6;
}

int SonarReturnData::getDataBytes() const
{
    return THCDecoder(packet[10], packet[11]);
}

QByteArray SonarReturnData::getEchoData() const
{
    return echo;
}

double SonarReturnData::getHeadPosition() const
{
    return headPos;
}

int SonarReturnData::getRange() const
{
    return range;
}

int SonarReturnData::getProfileRange() const
{
    return profileRange;
}

bool SonarReturnData::isPacketValid() const
{
    return valid;
}
