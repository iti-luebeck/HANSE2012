#include "global_sonar.h"


GlobalSonarNode::GlobalSonarNode(ros::NodeHandle n) : node(n){
    //advertize node for sonar with global points
    this->pub = this->node.advertise<geometry_msgs::PolygonStamped>("/globalsonar", 1000);

    ROS_INFO("Sonar node initialized");
}



void GlobalSonarNode::sonar_laser_update(const hanse_msgs::ELaserScan::ConstPtr& msg){
    //check if sonar scanning size has changed
    if(msg->laser_scan.ranges.size() != last_points.size()){
        //throw all points away
        last_points.clear();
        last_points.resize(msg->laser_scan.ranges.size());
        last_valid_points.clear();
        last_valid_points.resize(msg->laser_scan.ranges.size());
    }

    //
    Affine3d a = get_robot_transform();


    if(msg->laser_scan.ranges.at(msg->changed) >= 0){
        double angle = msg->laser_scan.angle_min + msg->laser_scan.angle_increment * msg->changed;

        //calculating x, y coordinates of laser scan
        Vector3d p(msg->laser_scan.ranges.at(msg->changed), 0, 0);
        AngleAxis<double> rotation(angle, Vector3d(0, 0, -1));
        p = rotation * p;

        //Converting to global coordinates
        p = a * p;

        //add point to polygon
        geometry_msgs::Point32 point;
        point.x = p(0);
        point.y = p(1);

        last_points[msg->changed] = point;
        last_valid_points[msg->changed] = true;
    } else {
        last_valid_points[msg->changed] = false;
    }

    std::vector<geometry_msgs::Point32> polygonPoints;
    for(unsigned int i = 0; i < last_points.size(); i++){
        if(last_valid_points[i]){
            polygonPoints.push_back(last_points[i]);
        }
    }

    //we could solve the TSP here, to have a nice sorting ;)

    //Publish StampedPolygon
    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();
    spolygon.polygon.points = polygonPoints;
    pub.publish(spolygon);
}


Affine3d GlobalSonarNode::get_robot_transform(){
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

    return a;
}


void GlobalSonarNode::pos_update(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //update last known pose
    last_pose = msg->pose;
}

int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "global_sonar");

    //create NodeHandle
    ros::NodeHandle n;
    
    GlobalSonarNode follow(n);

    //Subscribe to topic laser_scan (from sonar)
    ros::Subscriber esub_laser = n.subscribe<hanse_msgs::ELaserScan>("/hanse/sonar/e_laser_scan", 1000, boost::bind(&GlobalSonarNode::sonar_laser_update, &follow, _1));

    //Subscribe to the current position
    ros::Subscriber sub_pos = n.subscribe<geometry_msgs::PoseStamped>("/hanse/posemeter", 1000, boost::bind(&GlobalSonarNode::pos_update, &follow, _1));
    
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
