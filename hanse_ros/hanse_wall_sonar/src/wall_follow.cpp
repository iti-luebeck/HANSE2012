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
    algo = new wall_follow_shift_algo();

    ROS_INFO("Wall follow node initialized");

}



void WallFollowNode::sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg){

    //orientation of HANSE
    Quaterniond orientation(last_pose.orientation.w,
                            last_pose.orientation.x,
                            last_pose.orientation.y,
                            last_pose.orientation.z);

    //position of HANSE in global coordinates
    Translation3d position(last_pose.position.x,
                           last_pose.position.y,
                           last_pose.position.z);

    //Transformation from robot to global coordinates
    Affine3d a;
    a = position * orientation;


    transform_sonar_for_rviz(msg, a);


    std::cout << "debug result:\n";
    std::cout << last_pose.position.x;
    std::cout << "\n\n";
    // if we have no position information, we can't do anything
//    if(last_pose == NULL){
//        ROS_WARN("Yet waiting for robot pose. Skipping this sonar update.");
//        return;
//    }

    //let the algo calculate the position relative to robot
    Vector3d goal;
    Quaterniond goal_orientation;
    try {
       algo->sonar_laser_update(msg, goal, goal_orientation);
    } catch (std::runtime_error e) {
        ROS_ERROR_STREAM(e.what() << " Skipping...");
        return;
    }

    ROS_DEBUG_STREAM("The wall follow algo calculated\n" << goal << "\n");

    //FIXME use current pose
    geometry_msgs::PoseStamped spose;
    spose.header.frame_id = "/map";
    spose.header.stamp = ros::Time::now();

    goal = a * goal;

    spose.pose.position.x = goal(0);
    spose.pose.position.y = goal(1);
    spose.pose.position.z = goal(2);
    spose.pose.orientation.x = orientation.x();
    spose.pose.orientation.y = orientation.y();
    spose.pose.orientation.z = orientation.z();
    spose.pose.orientation.w = orientation.w();

    ROS_DEBUG_STREAM("Calculated pose is\n" << spose.pose << "\n");

    pub.publish(spose);
    debug_pub.publish(spose);
}

void WallFollowNode::transform_sonar_for_rviz(const sensor_msgs::LaserScan::ConstPtr& msg, Affine3d a){
    //Simulation polygon for laserscan visualisation
    std::vector<geometry_msgs::Point32> Polygon;

    //calculating Polygon
    for (unsigned int i = 0; i < msg->ranges.size(); i++){
        if(msg->ranges.at(i) > 0){
            double angle = msg->angle_min + msg->angle_increment * i;

            //calculating x, y coordinates of laser scan
            Vector3d p(msg->ranges.at(i), 0, 0);
            AngleAxis<double> rotation(angle, Vector3d(0, 0, -1));
            p = rotation * p;

            //Converting to global coordinates
            p = a * p;

            //add point to polygon
            geometry_msgs::Point32 point;
            point.x = p(0);
            point.y = p(1);
            Polygon.push_back(point);

        }
    }
    //Publish StampedPolygon
    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();
    spolygon.polygon.points=Polygon;
    debug_laser_pub.publish(spolygon);
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
