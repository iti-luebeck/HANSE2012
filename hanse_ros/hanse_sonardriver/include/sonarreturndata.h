#ifndef SONARRETURNDATA_H
#define SONARRETURNDATA_H

#include <QtCore>
#include "sonarswitchcommand.h"

class SonarReturnData
{
public:
    enum HeaderType
    {
        /**
          * 252 Data points in this package
          */
        IMX,

        /**
          * 500 Data points in this package
          */
        IGX,

        /**
          * Zarro Data points in this package
          */
        IPX
    };

public:

    SonarReturnData(const SonarSwitchCommand& cmd, QByteArray& returnDataPacket);

    SonarReturnData();
    SonarReturnData(const SonarReturnData& c);

    /**
      * Checks if this packet is valid, i.e. if the packet fulfills the spec
      * and none of the error bits in the status field are set.
      */
    virtual bool isPacketValid() const;

    /**
      * XXX: Some kind of error code?
      */
    bool isSwitchesAccepted() const;

    /**
      * XXX: Another error code?
      */
    bool isCharacterOverrun() const;

    /**
      * Returns the sensor range in meters.
      */
    int getRange() const;

    int getProfileRange() const;

    /**
      * The orientation of the sonar head when the meassurement was made.
      */
    double getHeadPosition() const;

    /**
      * Number of Data points in this packet.
      */
    int getDataBytes() const;

    /**
      * Returns true if the sonar head is currently moving in a clockwise direction
      */
    bool isCWDirection() const;

    /**
      * The actual echo data. XXX: actual format??
      */
    QByteArray getEchoData() const;

    /**
      * The unmodified data packet as received from the sonar
      */
    QByteArray packet;

    SonarReturnData& operator =(SonarReturnData);

    SonarSwitchCommand switchCommand;

protected:
    double headPos;
    bool valid;
    QByteArray echo;
    int range;
    int profileRange;

private:

    /**
      * Decodes a 14 bit integer from two consequtive packet bytes.
      *
      * The guys at Imagenex must have smoked some really nifty
      * shit when they devised this encoding.
      *
      * (see page 6/7 of the echo sounder protocol spec for
      * details.)
      */
    int THCDecoder(char byteLO, char byteHI) const;

    /**
      * Decodes the Head Position from two packet bytes.
      * very similar to THCDecoder(), only difference is that this
      * method masks another bit.
      */
    int THCHeadPosDecoder(char byteLO, char byteHI) const;

};

#endif // SONARRETURNDATA_H
