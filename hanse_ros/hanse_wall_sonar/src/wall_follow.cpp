#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "wall_follow.h"

class WallFollowNode {
public:
	WallFollowNode(ros::NodeHandle n) : _n(n)
	{
		_n.advertise<geometry_msgs::Twist>("/hanse/commands/cmd_vel", 1000);
	}
	
	void sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg){
    		ROS_INFO("I heard: [%d]", msg->ranges.size());
	}

    void pos_update(const geometry_msgs::PoseStamped::ConstPtr& msg){
            //ROS_INFO("I heard: [%d]", msg->ranges.size());
    }
	
private:
	ros::NodeHandle _n;
	ros::Publisher pub;
};

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
    
    WallFollowNode follow(n);

    //Subscribe to topic laser_scan (from sonar)
    ros::Subscriber sub_laser = n.subscribe("/hanse/sonar/laser_scan", 1000, boost::bind(&WallFollowNode::sonar_laser_update, &follow, _1));

    ros::Subscriber sub_pos = n.subscribe("/hanse/posemeter", 1000, boost::bind(&WallFollowNode::sonar_laser_update, &follow, _1));

    //TODO Publisher an Navigation anpassen!
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/hanse/commands/cmd_vel", 1000);
    

    ros::Rate loop_rate(10);


    ros::spin();

    return 0;
}
