#ifndef ECHOSOUNDERVIZ_H
#define ECHOSOUNDERVIZ_H

#include <map>
#include <cairomm/context.h>
#include <cairomm/surface.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "hanse_msgs/EchoSounder.h"

class EchoSounderViz
{
public:
  EchoSounderViz(ros::NodeHandle handle);

  void tick();

private:
  ros::NodeHandle nh;
  ros::Publisher publisher;
  ros::Subscriber subscriber;
  std::vector<hanse_msgs::EchoSounder> echoSounderMsgs;

  void callback(const hanse_msgs::EchoSounder &msg);
  sensor_msgs::Image cairoToRosImage(Cairo::RefPtr<Cairo::ImageSurface> surface);
};

#endif
