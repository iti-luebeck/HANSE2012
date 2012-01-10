#ifndef SONARVIZ_H
#define SONARVIZ_H

#include <map>
#include <cairomm/context.h>
#include <cairomm/surface.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "hanse_msgs/ScanningSonar.h"

class SonarViz
{
public:
  SonarViz(ros::NodeHandle handle);

  void tick();

private:
  ros::NodeHandle nh;
  ros::Publisher publisher;
  ros::Subscriber subscriber;
  std::map<double, hanse_msgs::ScanningSonar> sonarDataMap;
  double lastHeadPosition;

  void callback(const hanse_msgs::ScanningSonar &msg);
  sensor_msgs::Image cairoToRosImage(Cairo::RefPtr<Cairo::ImageSurface> surface);
};

#endif
