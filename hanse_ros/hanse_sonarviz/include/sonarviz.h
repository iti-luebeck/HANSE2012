#ifndef SONARVIZ_H
#define SONARVIZ_H

#include "ros/ros.h"

class SonarViz
{
public:
  SonarViz(ros::NodeHandle handle);

  void tick();

private:
  ros::NodeHandle nh;
  ros::Publisher publisher;

}

#endif
