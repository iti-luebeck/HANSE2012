#include "global_sonar.h"
//! \todo{TODO dynamic reconfigure}


GlobalSonarNode::GlobalSonarNode(ros::NodeHandle n) : node(n){
    //advertize node for sonar with global points
    this->pub = this->node.advertise<visualization_msgs::Marker>("/debug/sonar_global", 1000);
    ROS_INFO("Debug node initialized");
}



void GlobalSonarNode::sonar_laser_update(const sensor_msgs::LaserScan::ConstPtr& msg){
    //create point and color vectors for visualization
    std::vector<geometry_msgs::Point> points;
    std::vector<std_msgs::ColorRGBA> colors;
    //calculating vectors
    for (unsigned int i = 0; i < msg->ranges.size(); i++){
        if(msg->ranges.at(i) > 0){
            double angle = msg->angle_min + msg->angle_increment * i;

            //calculating x, y coordinates of laser scan
            Vector3d p(msg->ranges.at(i), 0, 0);
            AngleAxis<double> rotation(angle, Vector3d(0, 0, -1));
            p = rotation * p;

            //add point to point array
            geometry_msgs::Point point;
            point.x = p(0);
            point.y = p(1);
            points.push_back(point);

            //calc color
            std_msgs::ColorRGBA color;
            color.r = 0;
            color.g = 1;
            color.b = i * 1.0/msg->ranges.size();
            color.a = 1;

            colors.push_back(color);
        }
    }
    //Create marker
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.points = points;
    marker.pose = last_pose;
    marker.colors = colors;
    marker.color.a = 1;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;

    //choose typ
    //marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.type = visualization_msgs::Marker::POINTS;

    //choose scale
    marker.scale.x = .1;
    marker.scale.y = .1;
    marker.scale.z = .1;

    //publish marker
    this->pub.publish(marker);
}




void GlobalSonarNode::pos_update(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //update last known pose
    last_pose = msg->pose;
}

int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "debug_sonar");

    //create NodeHandle
    ros::NodeHandle n;
    
    GlobalSonarNode follow(n);

    //Subscribe to topic laser_scan (from sonar)
    ros::Subscriber sub_laser = n.subscribe<sensor_msgs::LaserScan>("/hanse/sonar/laser_scan", 1000, boost::bind(&GlobalSonarNode::sonar_laser_update, &follow, _1));

    //Subscribe to the current position
    ros::Subscriber sub_pos = n.subscribe<geometry_msgs::PoseStamped>("/hanse/posemeter", 1000, boost::bind(&GlobalSonarNode::pos_update, &follow, _1));
    
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
