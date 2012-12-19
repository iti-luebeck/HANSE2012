#include "wall_follow.h"
//! \todo{TODO allow reconfiguration to use without mars}
//! \todo{TODO subscriber in klasse auslagern?}
//! \todo{TODO Memory management?}

WallFollowNode::WallFollowNode(ros::NodeHandle n) : _n(n){
   //advertise to navigation goal
    pub = _n.advertise<geometry_msgs::PoseStamped>("/goal", 1000);

    debug_pub = _n.advertise<geometry_msgs::PoseStamped>("/debug/goal", 1000);

    debug_laser_pub = _n.advertise<geometry_msgs::PolygonStamped>("/debug/sonarPolygon", 1000);

    //TODO: think about memory ;)
    //algo = new wall_follow_shift_algo();
    algo = new wall_follow_fancy_algo();

    ROS_INFO("Wall follow node initialized");

}



void WallFollowNode::g_sonar_update(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    Vector3d goal;
    Quaterniond goal_orientation;
    try {
       algo->sonar_laser_update(msg, last_pose, goal, goal_orientation);
    } catch (std::runtime_error e) {
        ROS_ERROR_STREAM(e.what() << " Skipping...");
        return;
    }


    geometry_msgs::PoseStamped spose;
    spose.header.frame_id = "/map";
    spose.header.stamp = ros::Time::now();

    spose.pose.position.x = goal(0);
    spose.pose.position.y = goal(1);
    spose.pose.position.z = goal(2);
    spose.pose.orientation.x = goal_orientation.x();
    spose.pose.orientation.y = goal_orientation.y();
    spose.pose.orientation.z = goal_orientation.z();
    spose.pose.orientation.w = goal_orientation.w();

    //limit publish rate
    if(ros::Time::now().sec - last_goal_update > 5){
        pub.publish(spose);
        debug_pub.publish(spose);

        last_goal_update = ros::Time::now().sec;
    }
}

void WallFollowNode::pos_update(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //update last pose
    last_pose = msg->pose;
}

int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "wall_follow");

    //create NodeHandle
    ros::NodeHandle n;
    
    WallFollowNode follow(n);

    //Subscribe to topic laser_scan (from sonar)
    ros::Subscriber sub_laser = n.subscribe<geometry_msgs::PolygonStamped>("/oa_globalsonar", 1000, boost::bind(&WallFollowNode::g_sonar_update, &follow, _1));

    //Subscribe to the current position
    ros::Subscriber sub_pos = n.subscribe<geometry_msgs::PoseStamped>("/hanse/posemeter", 1000, boost::bind(&WallFollowNode::pos_update, &follow, _1));
    
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
