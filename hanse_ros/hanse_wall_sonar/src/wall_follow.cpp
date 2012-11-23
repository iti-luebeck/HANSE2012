#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "wall_follow.h"

void sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    //ROS_INFO(":D");
    ROS_INFO("I heard: [%d]", msg->ranges.size());
}

int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "wall_follow");

    //create NodeHandle
    ros::NodeHandle n;

    //Subscribe to topic laser_scan (from sonar)
    ros::Subscriber sub = n.subscribe("/hanse/sonar/laser_scan", 1000, sonar_laser_update);




    ros::Publisher 	pub = n.advertise<geometry_msgs::Twist>("/hanse/commands/cmd_vel", 1000);

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        geometry_msgs::Twist msg;

        msg.linear.x = .5;
        msg.angular.z = 1;

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }


    //ros::spin();

    return 0;
}
