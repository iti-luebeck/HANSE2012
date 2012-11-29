#include "wall_follow.h"

//! \todo{TODO allow reconfiguration to use without mars}
//! \todo{TODO subscriber in klasse auslagern?}


WallFollowNode::WallFollowNode(ros::NodeHandle n) : _n(n){
   //advertise to navigation goal
    pub = _n.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
}


void WallFollowNode::sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg){

}

void WallFollowNode::pos_update(const geometry_msgs::PoseStamped::ConstPtr& msg){

}

int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "wall_follow");

    //create NodeHandle
    ros::NodeHandle n;
    
    WallFollowNode follow(n);

    //Subscribe to topic laser_scan (from sonar)
    ros::Subscriber sub_laser = n.subscribe<sensor_msgs::LaserScan>("/hanse/sonar/laser_scan", 1000, boost::bind(&WallFollowNode::sonar_laser_update, &follow, _1));

    //Subscribe to the current position
    ros::Subscriber sub_pos = n.subscribe<geometry_msgs::PoseStamped>("/hanse/posemeter", 1000, boost::bind(&WallFollowNode::pos_update, &follow, _1));
    
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
