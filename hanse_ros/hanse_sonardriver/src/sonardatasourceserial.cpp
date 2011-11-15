#include <unistd.h>

#include "ros/ros.h"
#include "sonardatasourceserial.h"
#include "sonarreturndata.h"
#include "qextserialport.h"
#include "sonarswitchcommand.h"

SonarDataSourceSerial::SonarDataSourceSerial(QString portName)
{
    configurePort(portName);
}

const SonarReturnData SonarDataSourceSerial::getNextPacket()
{
    SonarSwitchCommand cmd;

    cmd.range = 50;
    cmd.startGain = 15;
    cmd.trainAngle = 70;
    cmd.sectorWidth = 120;
    cmd.stepSize = 2;
    cmd.pulseLength = 127;
    cmd.dataPoints = 25;
    cmd.switchDelay = 0;
    cmd.frequency = 1;


    QByteArray sendArray = cmd.toSerialCmd();

    port->write(sendArray);

    QByteArray retData;



    int expectedLength;
    if (cmd.dataPoints == 50)
        expectedLength = 513;
    else
        expectedLength = 265;

    int timeout = 1000; // if takes longer than a second, just drop the packet
    while(timeout>0 && (retData.length()==0 || retData[retData.length()-1] != (char)0xFC)) {
        QByteArray ret = port->read(expectedLength - retData.length());
        retData.append(ret);

        usleep(5000); timeout -= 5;
    }


    if (expectedLength - retData.length()>0) {
        ROS_DEBUG("Received less than expected: %i bytes missing; expected=%i", (int)(expectedLength - retData.length()), (int)expectedLength);
    } else {
        ROS_DEBUG("received full packet");
    }

    ROS_DEBUG("Received in total: %s", QString(retData.toHex()).toStdString().c_str());

    SonarReturnData d(cmd,retData);

    return d;
}

void SonarDataSourceSerial::configurePort(QString portName)
{
    PortSettings s;
    s.BaudRate = BAUD115200;
    s.DataBits = DATA_8;
    s.StopBits = STOP_1;
    s.Parity = PAR_NONE;
    s.Timeout_Millisec = 1;
    port = new QextSerialPort(portName, s, QextSerialPort::Polling);
    bool ret = port->open(QextSerialPort::ReadWrite);


    if (ret) {
        ROS_INFO("Opened serial port!");
    } else {
        ROS_ERROR("Could not open serial port %s", portName.toStdString().c_str());
    }

}

bool SonarDataSourceSerial::isOpen()
{
    return port && port->isOpen();
}

SonarDataSourceSerial::~SonarDataSourceSerial()
{
}

void SonarDataSourceSerial::stop()
{
//    logger->debug("Closing serial port.");
    if (port != NULL) {
        port->close();
        delete port;
        port = NULL;
    }
}
