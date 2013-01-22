#include "global_sonar.h"


GlobalSonarNode::GlobalSonarNode(ros::NodeHandle n) : node_(n){
    //subscribing to the topics corresponding to the current mode
    setupSubscribers();

    //advertize node for sonar with global points
    this->pub_ = this->node_.advertise<geometry_msgs::PolygonStamped>("sonar/global_sonar/polygon", 1000);


    config.store_time_sec_ = 7;

    ROS_INFO("Global sonar node initialized");


}

void GlobalSonarNode::sonarLaserUpdate(const hanse_msgs::ELaserScan::ConstPtr& msg){
    //check if sonar scanning size has changed
    if(msg->laser_scan.ranges.size() != last_points_.size()){
        //throw all points away
        last_points_.clear();
        last_points_.resize(msg->laser_scan.ranges.size());
        last_valid_points_.clear();
        last_valid_points_.resize(msg->laser_scan.ranges.size());
        //reset indexes
        last_newchange_ = 0;
        last_j_ = 0;
        //wait for next update
        return;
    }

     //let i the first index to read from incomming laser scan
    unsigned int i = (last_newchange_ + 1) % msg->laser_scan.ranges.size();
    //let j the first index to write to the saved laser scan
    //(j is orientation aware)
    int delta_j = (int) round(getRobotZRot() / (2*M_PI) * msg->laser_scan.ranges.size());
    unsigned int j = (last_newchange_ + 1 + delta_j) % msg->laser_scan.ranges.size();

    //spike removal
    //removes points if a low number of points (< 3)
    //will be left out between two updates
    if(abs(last_j_ - j) < 3){
        if (last_j_ < j){
            for(unsigned int q = last_j_; q < j; q++){
                last_valid_points_[q] = false;
            }
        } else {
            for(unsigned int q = j; q < last_j_; q++){
                last_valid_points_[q] = false;
            }
        }
    }

    // loop until we reached the last new data
    while(i != (msg->changed + 1) % msg->laser_scan.ranges.size()){
        //check point is valid
        if(msg->laser_scan.ranges[i] >= 0){
            double angle = msg->laser_scan.angle_min + msg->laser_scan.angle_increment * i;

            //add new point
            last_points_[j] = calculateGlobalPoint(angle, msg->laser_scan.ranges[i]);
            last_valid_points_[j] = true;

        } else {
            //invalidate point
            last_valid_points_[j] = false;
        }

        //circular increase i and j
        i = (i+1) % msg->laser_scan.ranges.size();
        j = (j+1) % msg->laser_scan.ranges.size();
    }




    //create polygon from valid points
    std::vector<geometry_msgs::Point32> polygonPoints;
    for(unsigned int i = 0; i < last_points_.size(); i++){
        if(last_valid_points_[i]){
            polygonPoints.push_back(last_points_[i]);
        }
    }

    //Publish StampedPolygon
    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();
    spolygon.polygon.points = polygonPoints;
    pub_.publish(spolygon);

    //let the last new change the actual new change
    last_newchange_ = msg->changed;
    //save the last index that changed the internal laser scan
    last_j_ = j;
}

void GlobalSonarNode::wallsUpdate(const hanse_msgs::WallDetection::ConstPtr& msg){

    double angle = msg->headPosition;
    uint32_t current_time = ros::Time::now().sec;

    if(msg->wallDetected){
        for(const double &distance : msg->distances){
            //creating and storing stamped position struct
            geometry_msgs::Point32 point = calculateGlobalPoint(angle, distance);
            posStamped pos;
            pos.pos_ = point;
            pos.sec_ = current_time;
            pos_list_.push_back(pos);
        }
    }

    //handling outdated data
    for(std::list<posStamped>::iterator it = pos_list_.begin() ; it != pos_list_.end(); ){
        if((*it).sec_ + config.store_time_sec_ < current_time){
            //remove old position from list
            it = pos_list_.erase(it);
        }else{
            //end iteration cause pos_list_ is ordered by inserttime
            it = pos_list_.end();
        }
    }


    std::cout << config.store_time_sec_ << "\n";


    //create polygon from pos_list_
    std::vector<geometry_msgs::Point32> polygonPoints;
    for(const posStamped &pos : pos_list_){
        polygonPoints.push_back(pos.pos_);
    }

    //Publish StampedPolygon
    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();
    spolygon.polygon.points = polygonPoints;
    pub_.publish(spolygon);
}


geometry_msgs::Point32 GlobalSonarNode::calculateGlobalPoint(double local_angle, double local_distance){
    //calculating x, y coordinates from local angle and distance
    Vector3d p(local_distance, 0, 0);
//inverting angle
    AngleAxis<double> rotation;
    if(simulation_mode_){
        rotation = AngleAxis<double>(-local_angle, Vector3d::UnitZ());
    }else{
        rotation = AngleAxis<double>(local_angle, Vector3d::UnitZ());
    }

    p = rotation * p;

    //Converting to global coordinates
    p = getRobotTransform() * p;

    //create and return point32
    geometry_msgs::Point32 point;
    point.x = p(0);
    point.y = p(1);
    point.z = p(2);
    return point;
}


Affine3d GlobalSonarNode::getRobotTransform(){
    //orientation of HANSE
    Quaterniond orientation(last_pose_.orientation.w,
                            last_pose_.orientation.x,
                            last_pose_.orientation.y,
                            last_pose_.orientation.z);

    //position of HANSE in global coordinates
    Translation3d position(last_pose_.position.x,
                           last_pose_.position.y,
                           last_pose_.position.z);

    //Transformation from robot to global coordinates
    Affine3d a;
    a = position * orientation;

    return a;
}

double GlobalSonarNode::getRobotZRot(){
    Quaterniond orientation(last_pose_.orientation.w,
                            last_pose_.orientation.x,
                            last_pose_.orientation.y,
                            last_pose_.orientation.z);
    Vector3d robot = Vector3d::UnitX();
    //transform a vector using the current orientation
    robot = orientation * robot;
    //calculate the rotation
    double robot_angle = atan2(robot[1], robot[0]);
    //normalize for a value between 0 and 2pi
    robot_angle = fmod(robot_angle + 2*M_PI,2*M_PI);

    return robot_angle;
}


void GlobalSonarNode::posUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //update last known pose
    last_pose_ = msg->pose;
}


void GlobalSonarNode::configCallback(hanse_wall_sonar::global_sonar_paramsConfig &config, uint32_t level){
    bool old_sim_mode = this->config.simulation_mode_;
    this->config = config;
    if (config.simulation_mode_ != old_sim_mode){
        setupSubscribers();
    }
}


void GlobalSonarNode::setupSubscribers(){
    sub_elaser.shutdown();
    sub_pos.shutdown();
    sub_walls.shutdown();
    if(config.simulation_mode_){
        //Subscribe to topic e_laser_scan (from sonar)
        sub_elaser = node_.subscribe<hanse_msgs::ELaserScan>("sonar/e_laser_scan", 1000, &GlobalSonarNode::sonarLaserUpdate, this);

        //Subscribe to the current position
        sub_pos = node_.subscribe<geometry_msgs::PoseStamped>("posemeter", 1000, &GlobalSonarNode::posUpdate, this);
    }else{
        //Subscribe to topic walls
        sub_walls = node_.subscribe<hanse_msgs::WallDetection>("sonar/scan/walls", 1000, &GlobalSonarNode::wallsUpdate, this);

        //Subscribe to the current position
        sub_pos = node_.subscribe<geometry_msgs::PoseStamped>("position/estimate", 1000, &GlobalSonarNode::posUpdate, this);
    }
}


int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "global_sonar");

    //create NodeHandle
    ros::NodeHandle n;
    
    GlobalSonarNode global_sonar(n);

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    dynamic_reconfigure::Server<hanse_wall_sonar::global_sonar_paramsConfig> dr_srv;
    dynamic_reconfigure::Server<hanse_wall_sonar::global_sonar_paramsConfig>::CallbackType cb;
    cb = boost::bind(&GlobalSonarNode::configCallback, &global_sonar, _1, _2);
    dr_srv.setCallback(cb);

    //TODO load parameters for dyn reconfigure

    
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
