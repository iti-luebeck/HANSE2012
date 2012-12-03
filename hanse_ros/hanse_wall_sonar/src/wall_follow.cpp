#include "wall_follow.h"
//! \todo{TODO allow reconfiguration to use without mars}
//! \todo{TODO subscriber in klasse auslagern?}
//! \todo{TODO Memory management?}

WallFollowNode::WallFollowNode(ros::NodeHandle n) : _n(n){
   //advertise to navigation goal
    pub = _n.advertise<geometry_msgs::PoseStamped>("/goal", 1000);

    debug_pub = _n.advertise<geometry_msgs::PoseStamped>("/debug/goal", 1000);

    //TODO: think about memory ;)
    algo = new wall_follow_shift_algo() ;

    ROS_INFO("Wall follow node initialized");
}


void WallFollowNode::sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg){
    std::cout << "debug result:\n";
    std::cout << last_pose.position.x;
    std::cout << "\n\n";
    // if we have no position information, we can't do anything
//    if(last_pose == NULL){
//        ROS_WARN("Yet waiting for robot pose. Skipping this sonar update.");
//        return;
//    }

    //let the algo calculate the position relative to robot
    Eigen::Vector2d r_goal;
    try {
       algo->sonar_laser_update(msg, r_goal);
    } catch (std::runtime_error e) {
        ROS_ERROR_STREAM(e.what() << " Skipping...");
        return;
    }
    Vector3d goal3d(r_goal(0), r_goal(1), 0);

    ROS_DEBUG_STREAM("The wall follow algo calculated\n" << r_goal << "\n");

    //FIXME use current pose
    geometry_msgs::PoseStamped spose;
    spose.header.frame_id = "/map";
    spose.header.stamp = ros::Time::now();

    ROS_DEBUG_STREAM("Calculated pose is\n" << spose.pose << "\n");


    Quaterniond orientation(last_pose.orientation.w,
                            last_pose.orientation.x,
                            last_pose.orientation.y,
                            last_pose.orientation.z);
    Translation3d position(last_pose.position.x,
                           last_pose.position.y,
                           last_pose.position.z);




    goal3d = Vector3d(1 ,0 , 0);

    goal3d = position * goal3d;
    spose.pose.position.x = goal3d(0);
    spose.pose.position.y = goal3d(1);
    spose.pose.orientation.x = orientation.x();
    spose.pose.orientation.y = orientation.y();
    spose.pose.orientation.z = orientation.z();
    spose.pose.orientation.w = orientation.w();

    //pub.publish(spose);
    debug_pub.publish(spose);
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
    ros::Subscriber sub_laser = n.subscribe<sensor_msgs::LaserScan>("/hanse/sonar/laser_scan", 1000, boost::bind(&WallFollowNode::sonar_laser_update, &follow, _1));

    //Subscribe to the current position
    ros::Subscriber sub_pos = n.subscribe<geometry_msgs::PoseStamped>("/hanse/posemeter", 1000, boost::bind(&WallFollowNode::pos_update, &follow, _1));
    
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
