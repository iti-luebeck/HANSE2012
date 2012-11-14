#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include "wall_follow.h"

void sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  //ROS_INFO(":D");
    ROS_INFO("I heard: [%f]", msg->ranges[16]);
}

int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "wall_follow");

    //create NodeHandle
    ros::NodeHandle n;

    //Subscribe to topic laser_scan (from sonar)
    ros::Subscriber sub = n.subscribe("/hanse/sonar/laser_scan", 1000, sonar_laser_update);

    ros::spin();

    return 0;
}
