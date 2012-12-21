#include "orientation_aware_global_sonar.h"


GlobalSonarNode::GlobalSonarNode(ros::NodeHandle n) : node(n), last_newchange(0), last_j(0){
    //advertize node for sonar with global points
    this->pub = this->node.advertise<geometry_msgs::PolygonStamped>("/oa_globalsonar", 1000);

    ROS_INFO("Orientation aware global sonar node initialized");
}



void GlobalSonarNode::sonar_laser_update(const hanse_msgs::ELaserScan::ConstPtr& msg){
    //check if sonar scanning size has changed
    if(msg->laser_scan.ranges.size() != last_points.size()){
        //throw all points away
        last_points.clear();
        last_points.resize(msg->laser_scan.ranges.size());
        last_valid_points.clear();
        last_valid_points.resize(msg->laser_scan.ranges.size());
        //reset indexes
        last_newchange = 0;
        last_j = 0;
        //wait for next update
        return;
    }

    //Affine object to transform from the robot coordinates to global coordinates
    Affine3d a = get_robot_transform();

    //let i the first index to read from incomming laser scan
    unsigned int i = (last_newchange + 1) % msg->laser_scan.ranges.size();
    //let j the first index to write to the saved laser scan
    //(j is orientation aware)
    int delta_j = (int) round(get_robot_z_rot() / (2*M_PI) * msg->laser_scan.ranges.size());
    unsigned int j = (last_newchange + 1 + delta_j) % msg->laser_scan.ranges.size();

    //spike removal
    //removes points if a low number of a low number of points (< 3)
    //will be left out between two updates
    if(abs(last_j - j) < 3){
        if (last_j < j){
            for(unsigned int q = last_j; q < j; q++){
                last_valid_points[q] = false;
            }
        } else {
            for(unsigned int q = j; q < last_j; q++){
                last_valid_points[q] = false;
            }
        }
    }

    // loop until we reached the last new data
    while(i != (msg->changed + 1) % msg->laser_scan.ranges.size()){
        //check point is valid
        if(msg->laser_scan.ranges[i] >= 0){
            double angle = msg->laser_scan.angle_min + msg->laser_scan.angle_increment * i;

            //calculating x, y coordinates of laser scan
            Vector3d p(msg->laser_scan.ranges[i], 0, 0);
            AngleAxis<double> rotation(-angle, Vector3d::UnitZ());
            p = rotation * p;

            //Converting to global coordinates
            p = a * p;

            //
            geometry_msgs::Point32 point;
            point.x = p(0);
            point.y = p(1);

            //add new point
            last_points[j] = point;
            last_valid_points[j] = true;

        } else {
            //invalidate point
            last_valid_points[j] = false;
        }

        //circular increase i and j
        i = (i+1) % msg->laser_scan.ranges.size();
        j = (j+1) % msg->laser_scan.ranges.size();
    }




    //create polygon from valid points
    std::vector<geometry_msgs::Point32> polygonPoints;
    for(unsigned int i = 0; i < last_points.size(); i++){
        if(last_valid_points[i]){
            polygonPoints.push_back(last_points[i]);
        }
    }

    //Publish StampedPolygon
    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();
    spolygon.polygon.points = polygonPoints;
    pub.publish(spolygon);

    //let the last new change the actual new change
    last_newchange = msg->changed;
    //save the last index that changed the internal laser scan
    last_j = j;
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

double GlobalSonarNode::get_robot_z_rot(){
    Quaterniond orientation(last_pose.orientation.w,
                            last_pose.orientation.x,
                            last_pose.orientation.y,
                            last_pose.orientation.z);
    Vector3d robot = Vector3d::UnitX();
    //transform a vector using the current orientation
    robot = orientation * robot;
    //calculate the rotation
    double robot_angle = atan2(robot[1], robot[0]);
    //normalize for a value between 0 and 2pi
    robot_angle = fmod(robot_angle + 2*M_PI,2*M_PI);

    return robot_angle;
}


void GlobalSonarNode::pos_update(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //update last known pose
    last_pose = msg->pose;
}

int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "orientation_aware_global_sonar");

    //create NodeHandle
    ros::NodeHandle n;
    
    GlobalSonarNode g_sonar(n);

    //Subscribe to topic laser_scan (from sonar)
    ros::Subscriber esub_laser = n.subscribe<hanse_msgs::ELaserScan>("/hanse/sonar/e_laser_scan", 1000, &GlobalSonarNode::sonar_laser_update, &g_sonar);

    //Subscribe to the current position
    ros::Subscriber sub_pos = n.subscribe<geometry_msgs::PoseStamped>("/hanse/posemeter", 1000, &GlobalSonarNode::pos_update, &g_sonar);
    
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
