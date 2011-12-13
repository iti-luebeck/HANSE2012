#ifndef SONARVIZ_H
#define SONARVIZ_H

#include <cairomm/context.h>
#include <cairomm/surface.h>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

class SonarViz
{
public:
  SonarViz(ros::NodeHandle handle);

  void tick();

private:
  ros::NodeHandle nh;
  ros::Publisher publisher;

  sensor_msgs::Image cairoToRosImage(Cairo::RefPtr<Cairo::ImageSurface> surface);
};

#endif
