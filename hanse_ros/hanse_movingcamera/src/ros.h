#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"
#include "Atmega644pHardware.h"

namespace ros
{
  typedef ros::NodeHandle_<Atmega644pHardware, 2, 2, 500, 500> NodeHandle;
}

#endif
