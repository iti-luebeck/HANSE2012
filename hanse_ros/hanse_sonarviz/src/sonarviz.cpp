#include "sonarviz.h"
#include "sensor_msgs/Image.h"

SonarViz::SonarViz(ros::NodeHandle handle) :
  nh(handle),
  publisher(handle.advertise<sensor_msgs::Image>("sonarviz", 1));
{
}

void SonarViz::tick()
{
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sonarviz");
  ros::NodeHandle n;
  ros::Rate r(1);

  SonarViz sonarViz;

  while (ros::ok()) {
    sonarViz.tick();
    r.sleep();
  }
}
