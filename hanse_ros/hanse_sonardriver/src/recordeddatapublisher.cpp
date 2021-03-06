#include <ros/ros.h>
#include "angles/angles.h"
#include "QtCore"
#include "hanse_msgs/ScanningSonar.h"
#include "sonarreturndata.h"
#include "scanningsonarswitchcommand.h"


const SonarReturnData getNextPacket(QDataStream& stream)
{
  quint8 nToReadIndex, trainAngle, pulseLength, profile, rovDepthUnits, headId, opFreq;
  quint16 totalBytes, nToRead, soundVelocity, rovDepth, rovHeading, rovTurnsCounter;
  
  stream.skipRawData(3); // skip "852"

  stream >> nToReadIndex;
  stream >> totalBytes;
  stream >> nToRead;

  stream.skipRawData(25); // skip date, time, hundredth of seconds
  stream.skipRawData(4); // skip 4 reserved bytes
  stream.skipRawData(1); // skip Dir, Xdcr, Mode, Step
  stream.skipRawData(1); // skip Start Gain
  stream.skipRawData(1); // skip (Sector Size)/3

  stream >> trainAngle;
  
  stream.skipRawData(3); // skip reserved bytes 41-43

  stream >> pulseLength;
  stream >> profile;
  stream >> soundVelocity;
  
  stream.skipRawData(32); //skip user text

  stream >> rovDepth;
  stream >> rovDepthUnits;
  stream >> rovHeading;
  stream >> rovTurnsCounter;
  stream >> opFreq;
  stream >> headId;

  stream.skipRawData(11);

  QByteArray retData;
  for(int i=0; i<nToRead; i++)
  {
    quint8 c;
    stream >> c;
    retData.append(c);
  }

  // skip zero fill
  stream.skipRawData(totalBytes - nToRead - 100); // .852 file header === 100bytes

  SonarReturnData d(ScanningSonarSwitchCommand(), retData);
  return d;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "sonar_recordeddatapublisher");
  ros::NodeHandle nh;
  std::string filename;

  if (!ros::param::get("~recording_filename", filename)) {
    ROS_ERROR("recording_filename parameter missing");
    return -1;
  }

  ros::Publisher publisher = nh.advertise<hanse_msgs::ScanningSonar>("sonar/scan", 1000);

  QFile file(QString::fromStdString(filename));
  if(!file.open(QFile::ReadOnly))
  {
    ROS_ERROR("could not open file %s", filename.c_str());
    return -1;
  }
  QDataStream stream(&file);

  while (ros::ok() && !stream.atEnd()) 
  {
    SonarReturnData returnData = getNextPacket(stream);

    hanse_msgs::ScanningSonar msg;

    QByteArray echoData = returnData.getEchoData();

    for (QByteArray::const_iterator i = echoData.begin(); i != echoData.end(); ++i) {
        msg.echoData.push_back(*i);
    }

    msg.header.stamp = ros::Time::now();
    msg.headPosition = angles::from_degrees(returnData.getHeadPosition());
    msg.range = returnData.getRange();
    msg.startGain = returnData.switchCommand.startGain;

    publisher.publish(msg);

    ROS_INFO("valid: %d,   headPos: %f", returnData.isPacketValid(), returnData.getHeadPosition());

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  return 0;
}
