#ifndef SONARDATASOURCESERIAL_H
#define SONARDATASOURCESERIAL_H

#include "sonarreturndata.h"

class QextSerialPort;

class SonarDataSourceSerial
{
public:
    SonarDataSourceSerial(QString portName);
    ~SonarDataSourceSerial();

    const SonarReturnData getNextPacket();

    bool isOpen();

    void stop();

private:
    QextSerialPort* port;

    void configurePort(QString portName);
};

#endif // SONARDATASOURCESERIAL_H
