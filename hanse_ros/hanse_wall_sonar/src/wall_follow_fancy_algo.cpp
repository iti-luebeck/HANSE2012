#include "wall_follow_fancy_algo.h"

using namespace Eigen;


WallFollowFancyAlgoNode::WallFollowFancyAlgoNode(ros::NodeHandle n):node_(n){

    pub_ = node_.advertise<geometry_msgs::PoseStamped>("/goal", 1000);
    pub_all_ = node_.advertise<geometry_msgs::PolygonStamped>("debug_fancy_poly", 1000);
    pub_path_ = node_.advertise<geometry_msgs::PolygonStamped>("debug_fancy_path", 1000);

    setupSubscribers();

}

void WallFollowFancyAlgoNode::sonarLaserUpdate(
        const geometry_msgs::PolygonStamped::ConstPtr& msg) throw (std::runtime_error)
{

    const Vector3d robot_position(last_pose_.position.x, last_pose_.position.y, last_pose_.position.z);
    const Quaterniond robot_oriantation(last_pose_.orientation.w,
                                  last_pose_.orientation.x,
                                  last_pose_.orientation.y,
                                  last_pose_.orientation.z);
    Vector3d robot_rot;
    robot_rot = robot_oriantation * Vector3d::UnitX();
    const double robot_yaw_angle = atan2(robot_rot(1), robot_rot(0));


    //create vector of Vector3d for global sonar points that are in front of the robot
    std::vector<Vector3d> global_front_sonar_points;
    for(const geometry_msgs::Point32 &p : msg->polygon.points){
        Vector3d vp(p.x, p.y, p.z);
        //if(!isBehindRobot(vp, robot_yaw_angle, robot_position)){
            //add point if it isn't behind the robot
            global_front_sonar_points.push_back(vp);
        //}
    }


    // create std::vector<Vector3d> of points in a circle
    std::vector<Vector3d> circ;
    for (unsigned int i = 0; i < 360; i+=config_.boundingcircle_angle_steps_) {
        const Vector3d dVector = config_.boundingcircle_radius_ * Vector3d::UnitX();
        AngleAxisd rotate(angles::from_degrees((double) i), Vector3d::UnitZ());
        circ.push_back(rotate * dVector);
    }
    std::list<Vector3d> result;
    for (const auto &p: global_front_sonar_points) {
        //create point circle for current point
        std::list<Vector3d> pCirc;
        for (const auto &c:circ) {
            Vector3d circ_point = c + p;
            pCirc.push_back(circ_point);
        }

        //filter unwanted points
        for (std::list<Vector3d>::iterator it = pCirc.begin(); it != pCirc.end(); ) {
            // remove all points that lie inside other circles
            const Vector3d &pc(*it);
            if(isInsideOtherCircle(config_.boundingcircle_radius_, global_front_sonar_points, pc)){
                //remove point if it lies inside another circle
                it = pCirc.erase(it);
            } else if(isBehindRobot(pc, robot_yaw_angle, robot_position)){
                //remove point if it is behind a robot
                it = pCirc.erase(it);
            } else {
                //keep point and continue with next point
                it++;
            }
        }
        result.insert(result.end(), pCirc.begin(), pCirc.end());
    }


    //Greedy Algorithm to calculate goal point and orientation

    //getting a list of points by looking at the next points that are close to each other
    //limit by a limit_lookahead_distancee, limit_lookahead_index_delta and limit_point_sdistance
    //TODO ensure that this algorithm won't go backward!

    const double limit_point_sdistance = pow(config_.limit_point_distance_, 2);
    const double limit_lookahead_sdistance = pow(config_.limit_lookahead_distance_, 2);

    std::vector<Vector3d> nearest_point_list = {robot_position};
    for(int i = 0; i < config_.limit_lookahead_index_delta_; i++){
        //use last point in nearest_point_list as reference point
        const Vector3d &ref_point(*(--nearest_point_list.end()));
        //initialize
        Vector3d const *nearestPoint = NULL;
        double nearest_sdistance = DBL_MAX;
        //search nearest point
        for(const Vector3d &p: result){
            const double p_sdistance = (p - ref_point).squaredNorm();
            if(p_sdistance < nearest_sdistance //is nearer
                    && (p-robot_position).squaredNorm() < limit_lookahead_sdistance //check lookahead distance limit
                    && std::find(nearest_point_list.begin(),nearest_point_list.end(),p)==nearest_point_list.end() //isn't already the nearest point list
                    && p_sdistance < limit_point_sdistance //check if distance between the two points is small enough
                    ){
                nearestPoint = &p;
                nearest_sdistance = (*nearestPoint - ref_point).squaredNorm();
            }
        }
        //check if we found a new Point
        if(nearest_sdistance != DBL_MAX){
            nearest_point_list.push_back(*nearestPoint);
        } else {
            //give up because there aren't any closer points
            break;
        }
    }
    //remove robot position
    nearest_point_list.erase(nearest_point_list.begin());

    //
    Vector3d sum(0, 0, 0);
    for(const auto &p:nearest_point_list){
        sum += p - robot_position;
    }


    Vector3d goal;
    goal = sum / sum.norm() + robot_position;

    Quaterniond goal_orientation;
    goal_orientation = AngleAxisd(atan2(sum(1), sum(0)), Vector3d::UnitZ());

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


    publishDebugInfo(result, nearest_point_list);

    //TODO: Auslagern / dyn reconf
    const unsigned int publishrate = 7;

    if(ros::Time::now().sec > publishrate  + last_goal_update_){
        pub_.publish(spose);
        //debug_pub_.publish(spose);

        last_goal_update_ = ros::Time::now().sec;
    }
}

bool WallFollowFancyAlgoNode::isBehindRobot(const Vector3d &p, const double &robot_yaw_angle, const Vector3d &robot_position)
{
    //calculate position relative to robot
    Vector3d p_reltative = p - robot_position;
    double p_angle = atan2(p_reltative(1), p_reltative(0));
    double diff = robot_yaw_angle - p_angle;
    diff = fmod(diff + 2.5 * M_PI, 2 * M_PI);
    //return true if the point p is behind
    return ((diff > M_PI));
}

bool WallFollowFancyAlgoNode::isInsideOtherCircle(const double &distance, const std::vector<Vector3d> &global_sonar_points, const Vector3d &pc)
{
    for (const Vector3d &q : global_sonar_points) {
        if ((q - pc).norm() < distance) {
            return true;
        }
    }
    return false;
}



void WallFollowFancyAlgoNode::publishDebugInfo(const std::list<Vector3d> &all,const std::vector<Vector3d> &path){
    geometry_msgs::PolygonStamped spolygon;
    spolygon.header.frame_id = "/map";
    spolygon.header.stamp = ros::Time::now();

    //create polygon from valid points
    for(const Vector3d &point: all){
        geometry_msgs::Point32 p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);
        spolygon.polygon.points.push_back(p);
    }
    pub_all_.publish(spolygon);


    //create polygon from valid points
    spolygon.polygon.points.clear();
    for(const Vector3d &point: path){
        geometry_msgs::Point32 p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);

        spolygon.polygon.points.push_back(p);
    }
    pub_path_.publish(spolygon);
}



void WallFollowFancyAlgoNode::configCallback(hanse_wall_sonar::wall_follow_fancy_algo_paramsConfig &config, uint32_t level){
    bool old_sim_mode = this->config_.simulation_mode_;
    this->config_ = config;
    if (config.simulation_mode_ != old_sim_mode){
        setupSubscribers();
    }

    ROS_INFO("%lf", config_.boundingcircle_radius_);
}

void WallFollowFancyAlgoNode::posUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg){
    //update last pose
    last_pose_ = msg->pose;
}

void WallFollowFancyAlgoNode::setupSubscribers(){
    sub_pos_.shutdown();
    sub_laser_.shutdown();

    //Subscribe to topic laser_scan (from sonar)
    sub_laser_ = node_.subscribe<geometry_msgs::PolygonStamped>("sonar/global_sonar/polygon", 1000, &WallFollowFancyAlgoNode::sonarLaserUpdate, this);

    if(config_.simulation_mode_){
        //Subscribe to the current position
        sub_pos_ = node_.subscribe<geometry_msgs::PoseStamped>("posemeter", 1000, &WallFollowFancyAlgoNode::posUpdate, this);
    }else {
        //Subscribe to the current position
        sub_pos_ = node_.subscribe<geometry_msgs::PoseStamped>("position/estimate", 1000, &WallFollowFancyAlgoNode::posUpdate, this);
    }
}


int main(int argc, char **argv)
{
    // init wall follow node
    ros::init(argc, argv, "wall_follow_fancy_algo");

    //create NodeHandle
    ros::NodeHandle n;

    WallFollowFancyAlgoNode wall_follow_fancy_algo(n);

    // Set up a dynamic reconfigure server.
    // This should be done before reading parameter server values.
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_fancy_algo_paramsConfig> dr_srv;
    dynamic_reconfigure::Server<hanse_wall_sonar::wall_follow_fancy_algo_paramsConfig>::CallbackType cb;
    cb = boost::bind(&WallFollowFancyAlgoNode::configCallback, &wall_follow_fancy_algo, _1, _2);
    dr_srv.setCallback(cb);

    //TODO load parameters for dyn reconfigure


    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}
